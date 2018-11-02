{ stdenv
, mkRosPackage
, python3Packages
, de_msgs
, robonomics_comm_msgs
, dji_sdk
, mavros
}:

let
  pname = "de_airsense";
  version = "0.0.0";

in mkRosPackage rec {
  name = "${pname}-${version}";

  src = ../de_airsense; 

  propagatedBuildInputs = with python3Packages; [ dji_sdk mavros robonomics_comm_msgs de_msgs web3 ipfsapi pyserial ];

  meta = with stdenv.lib; {
    description = "Drone Employee air pollution package";
    homepage = http://github.com/tuuzdu/de_robonomics;
    license = licenses.bsd3;
    maintainers = [ maintainers.tuuzdu ];
  };
}
