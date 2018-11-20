{ stdenv
, ros_comm
, mkRosPackage
, python3Packages
, robonomics_comm
, dji_sdk
, mavros
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "de_robonomics";
  version = "master";

  src = ./.;

  propagatedBuildInputs = with python3Packages;
  [ dji_sdk mavros robonomics_comm ros_comm web3 ipfsapi pyserial ];

  meta = with stdenv.lib; {
    description = "Drone Employee and Robonomics stack";
    homepage = http://github.com/tuuzdu/de_robonomics;
    license = licenses.bsd3;
    maintainers = [ maintainers.tuuzdu ];
  };
}