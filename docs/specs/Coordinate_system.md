В аполло используется СК ENU, в которой X указывает на восток, а Y на север. Аналогичная система координат используется и в [ROS](http://www.ros.org/reps/rep-0103.html).
Однако есть отличия по точке отсчета для ориентации (heading or yaw). В ROS угол отсчитывается от оси X (East) против часовой стрелки.
В Apollo ситуация двухсмысленная. Во-первых, в сообщении локализации они отсчитывают угол ориентации от оси Y (North). 
В связи с этим в [коде](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/gnss/parser/data_parser.cc#L268) где они формируют квантореон они отнимают 90 градусов, чтобы угол отсчитывался от оси Y

```
  // 2. orientation
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(ins->euler_angles().z() - 90 * DEG_TO_RAD_LOCAL,
                        Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-ins->euler_angles().y(), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(ins->euler_angles().x(), Eigen::Vector3d::UnitY());
```

Во-вторых, поле heading в сообщении localization::LocalizationEstimate необходимо указывать угол от оси X. Поэтому у них при получение heading из квантореона есть [добавление 90 градусов](https://github.com/ApolloAuto/apollo/blob/master/modules/common/math/quaternion.h#L62).
```
return NormalizeAngle(euler_angles.yaw() + M_PI_2);
```

В нашей системе локализации получается следующее:
1. GNSS система возвращает квантореон у которого угол yaw считается от оси Y (North). Думаю это связано с продольным положением антен на машине. Из-за этого в Rviz направление стрелки в одометрии отображется некорректно.
2. Для apollo именно такой отсчет от оси Y  и нужен, поэтому нам не нужно вычитать 90 градусов, как делается в родном драйвере.
3. При расчете heading мы используем стандартную функцию apollo::common::math::QuaternionToHeading, которая добавляет 90 градусов (т.е. преобразует heading к отсчету от оси X).
4. Модуль perception использует TF трансформации для вычисления положения других машин. Т.к. это стандартный механизм ROS, то он используется ориентацию от оси X и поэтому нам пришлось публиковать эти трансформации с +90 градусов относительно кванториона ориентации.

PS.  Если в сообщение локализации записать квантореон с отсчетом от оси X (как в ROS), то тогда начинает неверно работать модуль control, т.к. он неверно рассчитывает heading error в этом случае.

Issue, которые я нашел по этой теме.
https://github.com/ApolloAuto/apollo/issues/5713
https://github.com/ApolloAuto/apollo/issues/5676