{ lib ? import ./nix/lib.nix { }
}:

let
  callPackage = lib.nixpkgs.callPackage;

in rec {
  de_msgs = callPackage ./nix/msgs.nix { };
  de_airsense = callPackage ./nix/airsense.nix { 
  	inherit de_msgs;
  };
  de_robonomics = callPackage ./nix/default.nix { 
  	inherit de_msgs;
  	inherit de_airsense;
  };
}
