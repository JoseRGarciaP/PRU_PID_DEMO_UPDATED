The Device Tree include file is included in the following
Device Tree file:

am335x-bonegreen-wireless.dts

This is included by adding this line to the file:

#include "am335x-bonegreen-wireless-pid.dtsi"

The above line should be added to the END of the file!

NOT --> The "dtb-rebuilder" script by Robert Nelson is highly recommended:

https://github.com/RobertCNelson/dtb-rebuilder
