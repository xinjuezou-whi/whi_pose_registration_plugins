# whi_plain_pose_registration
![charging_logic-Page-3 drawio](https://github.com/xinjuezou-whi/whi_pose_registration_plugins/assets/72239958/bb126bb7-c3ab-46f9-a4dc-921f3a4c2b5c)

## Compile
For Ubuntu 18.04, error might be: "laser_geometry.h:47:10: fatal error: Eigen/Core: No such file or directory"
Replace "#include <Eigen/Core>" to "#include <eigen3/Eigen/Core>" in file "laser_geometry.h"
```
//#include <Eigen/Core>
#include <eigen3/Eigen/Core> // WHI
```
