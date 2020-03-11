# 英雄机器人更新日志

## Date:         2019.12.5

Author:     张祥龙
Version:    `UP1.1`
版本更新：修复代码中YAW疯转，取最大值的BUG，CAN2移植中断BUG

## Date:         2019.12.5

Author:     张祥龙
Version:    `UP1.2`
版本更新：CAN2移植中断BUG进一步修正，收发实验完成，CAN2接收遥控器代码完成

## Date:         2019.12.5

Author:     张祥龙
Version:    `UP2.1`
版本更新：基于定时器3的操作系统完成，大幅修改代码，删除CubeMX文件

## Date:         2020.2.16

Author:     张祥龙
Version:    `UP3.1`
版本更新：改动较大，说明文档还没写完

## Date:         2020.2.25

Author:     杨玉书
Version:    `UP3.1hero`
版本更新：修改了英雄车摩擦轮发射部分代码，结构还是和之前英雄车测试的代码一致，由于pid文件的删除，
现在新增了pid计算函数，在shoot.c最底部，但是shoot.h文件有一个bug，定义在pid.h中的pid-t结构体在shoot.h中找不到

## Date:         2020.2.25

Author:     吴凯
Version:    `UP3.1.1hero`
版本更新：修复上一版本shoot.h的bug
新增了pid模糊计算函数，在shoot文件夹下的blurry_pid.c,但需要设置的参数过多，同时不知道代码是否有问题，无法没办法做测试，所以正式版本暂时还是用基本pid。

## Date:         2020.2.25

Author:     吴凯
Version:    `UP3.1.1hero`
版本更新：修复上一版本shoot.h的bug
新增了pid模糊计算函数，在shoot文件夹下的blurry_pid.c,但需要设置的参数过多，同时不知道代码是否有问题，无法没办法做测试，所以正式版本暂时还是用基本pid。

## Date:         2020.3.11

Author:     马立伟
Version:    `UP3.2.1hero`
版本更新：增加卡单处理，自动检测卡单并自动倒转。
代码位于revolver.c，`void REVOL_PositStuck(void)`这个函数，但时间相关的参数还需要调整。