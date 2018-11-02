{ stdenv
, mkRosPackage
, python3Packages
, ros_comm
}:

let
  pname = "de_msgs";
  version = "0.0.0";

in mkRosPackage rec {
  name = "${pname}-${version}";

  src = ../de_msgs; 

  propagatedBuildInputs = with python3Packages; [ ros_comm ];

  meta = with stdenv.lib; {
    description = "Drone Employee messages";
    homepage = http://github.com/tuuzdu/de_robonomics;
    license = licenses.bsd3;
    maintainers = [ maintainers.tuuzdu ];
  };
}