# -*- coding: utf-8 -*-

from robonomics_lighthouse.msg import Ask, Bid
from std_msgs.msg import String
from std_srvs.srv import Empty
from web3 import Web3, HTTPProvider
from threading import Thread
from urllib.parse import urlparse
import rospy, ipfsapi


class Agent:

    current_measurement = None

    def __init__(self):
        rospy.init_node('de_airsense_agent')

        self.model = rospy.get_param('~model')
        self.token = rospy.get_param('~token')
        self.bid_lifetime = rospy.get_param('~bid_lifetime')
        self.web3 = Web3(HTTPProvider(rospy.get_param('~web3_http_provider')))
        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))
        self.signing_bid_pub = rospy.Publisher('liability/infochan/signing/bid', Bid, queue_size=10)

        def incoming_ask(ask_msg):
            rospy.loginfo('Incoming ask: ' + str(ask_msg))
            if ask_msg.model == self.model and ask_msg.token == self.token:
                rospy.loginfo('Incoming ask with right model and token')

                def objective_check():
                    rospy.loginfo('Waiting for objective file from IPFS...')
                    self.ipfs.ls(ask_msg.objective)
                    # msg_type = list(bag.get_type_and_topic_info()[0].values())[0]
                check_thread = Thread(target=objective_check)
                check_thread.start()
                check_thread.join(60)
                if check_thread.is_alive():
                    rospy.logwarn('Timeout of waiting objective file: ' + str(ask_msg.objective))
                    rospy.logwarn('Skip incoming ask')
                    return

                self.make_bid(ask_msg)
            else:
                rospy.logwarn('Incoming ask with wrong model and token, skip')
        rospy.Subscriber('liability/infochan/incoming/ask', Ask, incoming_ask)

        def measurements(hash_msg):
            self.current_measurement = hash_msg.data
            rospy.loginfo('Received measurements in IPFS file: ' + hash_msg.data)
        rospy.Subscriber('de_airsense_waspmote_ipfs/result/measurements', String, measurements)

        rospy.wait_for_service('liability/finish')
        self.finish_srv = rospy.ServiceProxy('liability/finish', Empty)

        Thread(target=self.process, daemon=True).start()

    def make_bid(self, incoming_ask):
        rospy.loginfo('Making bid...')
        bid = Bid()
        bid.model = self.model
        bid.objective = incoming_ask.objective
        bid.token = self.token
        bid.cost = incoming_ask.cost
        bid.lighthouseFee = 0
        bid.deadline = self.web3.eth.getBlock('latest').number + self.bid_lifetime
        self.signing_bid_pub.publish(bid)

    def process(self):
        while True:
            while not self.current_measurement:
                rospy.sleep(1)
            self.current_measurement = None
            self.finish_srv()
            rospy.loginfo('Liability finished')
    
    def spin(self):
        rospy.spin()
