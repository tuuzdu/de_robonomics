{ stdenv
, mkRosPackage
, de_msgs
, de_airsense
}:

let
  pname = "de_robonomics";
  version = "0.0.0";

in mkRosPackage rec {
  name = "${pname}-${version}";

  src = ../de_robonomics;

  propagatedBuildInputs =
  [ de_msgs
    de_airsense ];

  meta = with stdenv.lib; {
    description = "Drone Employee stack";
    homepage = http://github.com/tuuzdu/de_robonomics;
    license = licenses.bsd3;
    maintainers = [ maintainers.tuuzdu ];
  };
}
