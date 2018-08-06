#!/usr/bin/env python
# -*- coding: utf-8 -*-


from sys import version_info
if version_info[0] < 3:
    raise Exception('python3 required')
try:
    import rosbag
    import rospy
except ImportError as e:
    link = 'https://github.com/OTL/cozmo_driver#super-hack-to-run-rospy-from-python3'
    raise(ImportError('How to run ROS on python3: ' + link)) from e
from de_msgs.msg import Mission, Waypoint
import ipfsapi
from flask import Flask, request
from flask_restful import Resource, Api
from flask.json import jsonify
from flask_cors import CORS
from urllib.parse import urlparse
import tempfile
import os


rospy.init_node('objective_composer')

ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

app = Flask(__name__)
CORS(app)
api = Api(app)

tmp_dir = rospy.get_param('~tmp_dir', '/tmp/agent')
if not os.path.exists(tmp_dir):
    os.makedirs(tmp_dir)


class ComposeObjective(Resource): # ui input to agent liability objective
    def post(self):
        """request example:
           $ curl -H "Content-type: application/json" \
                  -X POST https://0.0.0.0:8888/ \
                  -d '{"lat":"49.368550","lng":"75.403660","alt":"100","time":"5"}'
        """
        rospy.loginfo('Composer request for: ' + str(request.get_json()))
        bag_name = tmp_dir + '/objective-' + next(tempfile._get_candidate_names()) + '.bag'

        with rosbag.Bag(bag_name, 'w') as bag:
            mission = Mission()
            waypoint = Waypoint()
            waypoint.latitude   = float(request.get_json()['lat'])
            waypoint.longitude = float(request.get_json()['lng'])
            waypoint.altitude = float(request.get_json().get('alt', 100))
            waypoint.staytime = int(request.get_json().get('time', 5))
            mission.waypoints   = [waypoint, waypoint]
            bag.write('/objective/mission', mission)

        rospy.loginfo('Bag: ' + bag_name)
        ipfs_response = ipfs.add(bag_name)
        try:
            objective_hash = ipfs_response['Hash']
        except TypeError:
            rospy.logwarn('IPFS add proceeding error: %s', ipfs_response[1]['Message'])
            objective_hash = ipfs.add(bag_name)[0]['Hash']
        rospy.loginfo('Objective: ' + objective_hash)
        return jsonify({'objective': objective_hash})

api.add_resource(ComposeObjective, '/get_objective')


server_address = urlparse(rospy.get_param('~server_address')).netloc.split(':')
app.run(host=server_address[0],
        port=int(server_address[1]),
        ssl_context=(rospy.get_param('~certfile'), rospy.get_param('~keyfile')))
rospy.spin()
