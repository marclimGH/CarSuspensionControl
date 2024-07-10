package SuspensionSystem
  import SI = Modelica.Units.SI;

  package Components
    class RoadProfile
      parameter Real roadRoughness = 3 "Road height StdDeviation in cm";
      Modelica.Mechanics.MultiBody.Joints.Prismatic Road(animation = true,boxColor = {140, 140, 140}, boxHeight = 1, boxWidth = 0.3, n = {0, 1, 0}, s(start = 0.5), useAxisFlange = true) annotation(
        Placement(visible = true, transformation(origin = {38, -16}, extent = {{-30, -30}, {30, 30}}, rotation = 90)));
      Modelica.Mechanics.Translational.Sources.Position position(a(fixed = false), exact = false, v(fixed = false)) annotation(
        Placement(visible = true, transformation(origin = {0, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b zR annotation(
        Placement(visible = true, transformation(origin = {38, 96}, extent = {{-16, -16}, {16, 16}}, rotation = 90), iconTransformation(origin = {-2, 96}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
      Modelica.Blocks.Continuous.Filter LPF(f_cut = 1, filterType = Modelica.Blocks.Types.FilterType.LowPass, gain = roadRoughness, order = 2) annotation(
        Placement(visible = true, transformation(origin = {-90, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Noise.NormalNoise normalNoise(enableNoise = true, mu = 0, samplePeriod = 0.01, sigma = 0.05, startTime = 0.1, useAutomaticLocalSeed = false, useGlobalSeed = false) annotation(
        Placement(visible = true, transformation(origin = {-120, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a world_a annotation(
        Placement(visible = true, transformation(origin = {-98, -72}, extent = {{-16, -16}, {16, 16}}, rotation = 180), iconTransformation(origin = {-100, -66}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation y(r = {0, -0.5, 0}) annotation(
        Placement(visible = true, transformation(origin = {38, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput RoadRoughness annotation(
        Placement(visible = true, transformation(origin = {-108, -16}, extent = {{-14, -14}, {14, 14}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Product product annotation(
        Placement(visible = true, transformation(origin = {-58, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add annotation(
        Placement(visible = true, transformation(origin = {-28, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression offset(y = 0.5)  annotation(
        Placement(visible = true, transformation(origin = {-64, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
  connect(position.support, Road.support) annotation(
        Line(points = {{0, -2}, {0, -28}, {20, -28}}, color = {0, 127, 0}));
  connect(position.flange, Road.axis) annotation(
        Line(points = {{10, 8}, {20, 8}}, color = {0, 127, 0}));
      connect(zR, Road.frame_b) annotation(
        Line(points = {{38, 96}, {38, 14}}));
  connect(normalNoise.y, LPF.u) annotation(
        Line(points = {{-109, 20}, {-102, 20}}, color = {0, 0, 127}));
      connect(world_a, y.frame_a) annotation(
        Line(points = {{-98, -72}, {-16, -72}, {-16, -96}, {38, -96}, {38, -86}}));
      connect(y.frame_b, Road.frame_a) annotation(
        Line(points = {{38, -66}, {38, -46}}, color = {95, 95, 95}));
  connect(LPF.y, product.u1) annotation(
        Line(points = {{-79, 20}, {-71, 20}}, color = {0, 0, 127}));
  connect(RoadRoughness, product.u2) annotation(
        Line(points = {{-108, -16}, {-76, -16}, {-76, 8}, {-70, 8}}, color = {0, 0, 127}));
  connect(add.y, position.s_ref) annotation(
        Line(points = {{-16, 8}, {-12, 8}}, color = {0, 0, 127}));
  connect(product.y, add.u1) annotation(
        Line(points = {{-46, 14}, {-40, 14}}, color = {0, 0, 127}));
  connect(offset.y, add.u2) annotation(
        Line(points = {{-52, -28}, {-46, -28}, {-46, 2}, {-40, 2}}, color = {0, 0, 127}));
    end RoadProfile;

    model Wheel
      parameter SI.Mass mT = 50 "suspended mass (Tyre)";
      //parameter Modelica.Units.SI.TranslationalDampingConstant cT;
      parameter SI.TranslationalSpringConstant kT = 250000;
      parameter Modelica.Units.SI.Length wr = 0.35 "wheel radius";
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a zR annotation(
        Placement(visible = true, transformation(origin = {0, -98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -94}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
      Modelica.Mechanics.MultiBody.Parts.BodyShape WheelMass(animateSphere = false, color = {70, 70, 70}, height = 0.3, length = wr * 0.4, lengthDirection = {0, 0, 1}, m = mT, r = {0, wr * 0.65, 0}, r_0(start = {0, wr, 0}), r_CM = {0, wr * 0.65, 0}, r_shape = {0, wr * 0.65, 0}, width = wr * 2) annotation(
        Placement(visible = true, transformation(origin = {0, 24}, extent = {{-26, -26}, {26, 26}}, rotation = 90)));
      Modelica.Mechanics.MultiBody.Joints.Prismatic TyreElasticity(animation = true, boxColor = {50, 255, 50}, boxHeight = 0.3, n = {0, 1, 0}, s(start = wr / 5), useAxisFlange = true) annotation(
        Placement(visible = true, transformation(origin = {1, -59}, extent = {{-21, -21}, {21, 21}}, rotation = 90)));
      Modelica.Mechanics.Translational.Components.Spring TyreSpring(c = kT, s_rel(fixed = true, start = wr * 0.28), s_rel0 = wr * 0.35) annotation(
        Placement(visible = true, transformation(origin = {-52, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Mechanics.MultiBody.Parts.BodyCylinder clip(diameter = 0.05, innerDiameter = 0.03, r = {0, 0.1, -0.1}) annotation(
        Placement(visible = true, transformation(origin = {14, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b zT annotation(
        Placement(visible = true, transformation(origin = {28, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
    equation
      connect(TyreElasticity.frame_a, zR) annotation(
        Line(points = {{1, -80}, {0, -80}, {0, -98}}, color = {95, 95, 95}));
      connect(TyreElasticity.frame_b, WheelMass.frame_a) annotation(
        Line(points = {{1, -38}, {1, -9}, {0, -9}, {0, -2}}));
      connect(TyreElasticity.support, TyreSpring.flange_a) annotation(
        Line(points = {{-11.6, -67.4}, {-51.6, -67.4}, {-51.6, -65.4}}, color = {0, 127, 0}));
      connect(TyreSpring.flange_b, TyreElasticity.axis) annotation(
        Line(points = {{-52, -46}, {-50, -46}, {-50, -42}, {-12, -42}}, color = {0, 127, 0}));
      connect(clip.frame_b, zT) annotation(
        Line(points = {{24, 76}, {28, 76}, {28, 100}}, color = {95, 95, 95}));
      connect(clip.frame_a, WheelMass.frame_b) annotation(
        Line(points = {{4, 76}, {0, 76}, {0, 50}}));
    end Wheel;

    package QuarterCar

      model QuarterCarModelBase
        parameter SI.Mass mB = 400 "body mass";
        parameter SI.Mass mT = 50 "suspended mass (Tyre)";
        parameter SI.Length unstretchedLen = 1 "suspension unstretched length";
        parameter Modelica.Units.SI.TranslationalDampingConstant cB = 1300;
        parameter Modelica.Units.SI.TranslationalSpringConstant kB = 20000;
        //parameter Modelica.Units.SI.TranslationalDampingConstant cT;
        parameter Modelica.Units.SI.TranslationalSpringConstant kT = 250000;
        Modelica.Mechanics.MultiBody.Interfaces.Frame_a zR annotation(
          Placement(visible = true, transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {-2, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
        Modelica.Mechanics.MultiBody.Parts.PointMass bodyMass(m = mB, r_0(start = {0, 1.2, 0}), sphereDiameter = 0.3, stateSelect = StateSelect.default) annotation(
          Placement(visible = true, transformation(origin = {0, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Wheel wheel annotation(
          Placement(visible = true, transformation(origin = {1.77636e-15, -50}, extent = {{-18, -18}, {18, 18}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b z annotation(
          Placement(visible = true, transformation(origin = {-2, 130}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {-6, 94}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
        Modelica.Mechanics.MultiBody.Forces.Spring spring(c = kB, s_unstretched = unstretchedLen) annotation(
          Placement(visible = true, transformation(origin = {31, 23}, extent = {{-17, -17}, {17, 17}}, rotation = 90)));
        replaceable Modelica.Mechanics.MultiBody.Forces.Damper damper(d = cB) annotation(
          Placement(visible = true, transformation(origin = {-44, 20}, extent = {{-18, -18}, {18, 18}}, rotation = 90)));
        Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity z_vel_sensor annotation(
          Placement(visible = true, transformation(origin = {64, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity zW_vel_sensor annotation(
          Placement(visible = true, transformation(origin = {69, -35}, extent = {{-13, -13}, {13, 13}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput z_vel annotation(
          Placement(visible = true, transformation(origin = {106, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput zW_vel annotation(
          Placement(visible = true, transformation(origin = {110, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput inDampingControl annotation(
          Placement(visible = true, transformation(origin = {-98, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-98, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      equation
        connect(bodyMass.frame_a, z) annotation(
          Line(points = {{0, 70}, {-2, 70}, {-2, 130}}, color = {95, 95, 95}));
        connect(wheel.zR, zR) annotation(
          Line(points = {{0, -66}, {0, -100}}, color = {95, 95, 95}));
        connect(wheel.zT, spring.frame_a) annotation(
          Line(points = {{6, -32}, {31, -32}, {31, 6}}, color = {95, 95, 95}));
        connect(spring.frame_b, bodyMass.frame_a) annotation(
          Line(points = {{31, 40}, {0, 40}, {0, 70}}));
        connect(wheel.zT, damper.frame_a) annotation(
          Line(points = {{6, -32}, {2, -32}, {2, -16}, {-44, -16}, {-44, 2}}));
        connect(damper.frame_b, bodyMass.frame_a) annotation(
          Line(points = {{-44, 38}, {-32, 38}, {-32, 40}, {0, 40}, {0, 70}}));
        connect(z_vel_sensor.frame_a, bodyMass.frame_a) annotation(
          Line(points = {{54, 70}, {0, 70}}, color = {95, 95, 95}));
        connect(wheel.zT, zW_vel_sensor.frame_a) annotation(
          Line(points = {{6, -32}, {32, -32}, {32, -35}, {56, -35}}, color = {95, 95, 95}));
        connect(z_vel_sensor.v[2], z_vel) annotation(
          Line(points = {{76, 70}, {106, 70}}, color = {0, 0, 127}));
        connect(zW_vel_sensor.v[2], zW_vel) annotation(
          Line(points = {{84, -34}, {110, -34}}, color = {0, 0, 127}));
      end QuarterCarModelBase;

      model QuarterCarModelPassive
        parameter SI.TranslationalDampingConstant d = 1000;
        parameter Modelica.Units.SI.Length unstretchedLen = 1;
        extends QuarterCar.QuarterCarModelBase(damper.d = d);
      equation
        inDampingControl = 0;
      end QuarterCarModelPassive;

      class QuarteCarModelActiveDamping
        parameter SI.TranslationalDampingConstant dmax = 2000;
        parameter SI.TranslationalDampingConstant dmin = 500;
        parameter Modelica.Units.SI.Length unstretchedLen = 1;
        extends QuarterCarModelBase(redeclare Components.Damper.ActiveDamper3d damper(dmax = dmax, dmin = dmin) );
      equation
        inDampingControl = damper.inDampingControl;
      end QuarteCarModelActiveDamping;
    end QuarterCar;

    package Damper
      model ActiveDamper3d
        parameter SI.TranslationalDampingConstant dmax = 2000;
        parameter SI.TranslationalDampingConstant dmin = 500;
        parameter Real d;
        parameter Modelica.Units.SI.Length unstretchedLen = 1;
        Components.Damper.ActiveDamper activeDamper(dmax = dmax, dmin = dmin) annotation(
          Placement(visible = true, transformation(origin = {-37, -3}, extent = {{-19, -19}, {19, 19}}, rotation = 90)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
          Placement(visible = true, transformation(origin = {2, -92}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {98, -4}, extent = {{-16, -16}, {16, 16}}, rotation = 180)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
          Placement(visible = true, transformation(origin = {-2, 92}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {-94, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput inDampingControl annotation(
          Placement(visible = true, transformation(origin = {-104, 4}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {18, 86}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
        Modelica.Mechanics.MultiBody.Forces.LineForceWithMass lineForceWithMass annotation(
          Placement(visible = true, transformation(origin = {39, -1}, extent = {{-33, -33}, {33, 33}}, rotation = 90)));
      equation
        connect(inDampingControl, activeDamper.dampingControl) annotation(
          Line(points = {{-104, 4}, {-52, 4}}, color = {255, 0, 255}));
        connect(frame_b, lineForceWithMass.frame_b) annotation(
          Line(points = {{-2, 92}, {-6, 92}, {-6, 50}, {40, 50}, {40, 32}}));
        connect(frame_a, lineForceWithMass.frame_a) annotation(
          Line(points = {{2, -92}, {2, -58}, {40, -58}, {40, -34}}));
        connect(activeDamper.flange_b, lineForceWithMass.flange_b) annotation(
          Line(points = {{-36, 16}, {6, 16}, {6, 18}}, color = {0, 127, 0}));
        connect(activeDamper.flange_a, lineForceWithMass.flange_a) annotation(
          Line(points = {{-36, -22}, {-15, -22}, {-15, -20}, {6, -20}}, color = {0, 127, 0}));
        connect(inDampingControl, activeDamper.dampingControl) annotation(
          Line(points = {{-104, 4}, {-52, 4}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Line(points = {{60, -30}, {-60, -30}, {-60, 30}, {60, 30}}, color = {0, 127, 0}), Line(visible = false, points = {{-100, -100}, {-100, -20}, {-14, -20}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Line(points = {{-90, 0}, {100, 0}}, color = {0, 127, 0}), Text(extent = {{-150, -45}, {150, -75}}, textString = "d=%d"), Line(points = {{-60, -30}, {-60, 30}}), Rectangle(lineColor = {0, 127, 0}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-60, 30}, {30, -30}})}));
      end ActiveDamper3d;

      model ActiveDamper
        extends Modelica.Mechanics.Translational.Interfaces.PartialCompliantWithRelativeStates;
        parameter SI.TranslationalDampingConstant dmin= 500;
        parameter SI.TranslationalDampingConstant dmax= 2000;
        SI.TranslationalDampingConstant d(start = dmax);
        SI.Force f(start = 0);
        Modelica.Blocks.Interfaces.RealInput dampingControl annotation(
          Placement(visible = true, transformation(origin = {-108, 58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {40, 84}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
      equation
        d = dmin + (dmax - dmin) * dampingControl;
        f = d * v_rel;
        annotation(
          Icon(graphics = {Line(points = {{60, -30}, {-60, -30}, {-60, 30}, {60, 30}}, color = {0, 127, 0}), Line(visible = false, points = {{-100, -100}, {-100, -20}, {-14, -20}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Line(points = {{-90, 0}, {100, 0}}, color = {0, 127, 0}), Text(extent = {{-150, -45}, {150, -75}}, textString = "d=%d"), Line(points = {{-60, -30}, {-60, 30}}), Rectangle(lineColor = {0, 127, 0}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-60, 30}, {30, -30}})}));
      end ActiveDamper;
    end Damper;

    package Controller
      class ControllerBase
        Modelica.Blocks.Interfaces.RealOutput outDampingControl annotation(
          Placement(visible = true, transformation(origin = {105, 13}, extent = {{-17, -17}, {17, 17}}, rotation = 0), iconTransformation(origin = {98, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput z_vel_in annotation(
          Placement(visible = true, transformation(origin = {-106, 70}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-98, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput zW_vel_in annotation(
          Placement(visible = true, transformation(origin = {-106, 6}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-98, -8}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      equation

      end ControllerBase;

      model ControllerADD "Acceleration Driven Control"
        extends ControllerBase;
        Modelica.Units.SI.Acceleration z_acc;
        Real switchLogic;
        parameter Real steepness = 10 "Steepness of the transition";
      equation
        z_acc = der(z_vel_in);
        switchLogic = z_acc * (z_vel_in - zW_vel_in);
        outDampingControl = 0.5 * (1 + tanh(steepness * switchLogic));
      end ControllerADD;
      
      model ControllerSHADD "Acceleration Driven Control"
        extends ControllerBase;
        Modelica.Units.SI.Acceleration z_acc;
        Real ADD,SH,switchLogic;
        parameter Real steepness = 10 "Steepness of the transition";
        parameter Real kSH = 1;
        parameter Real kADD = 1;
      equation
        z_acc = der(z_vel_in);
        ADD = z_acc * (z_vel_in - zW_vel_in);
        SH =  z_vel_in * (z_vel_in - zW_vel_in);
        switchLogic = kSH*SH + kADD*ADD ;
        outDampingControl = 0.5 * (1 + tanh(steepness * switchLogic));
      end ControllerSHADD;
    end Controller;

    partial model BaseSuspension
      Real accSquared;
      inner SuspensionSystem.Components.RoadProfile roadProfile annotation(
        Placement(visible = true, transformation(origin = {-1, -63}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
      inner replaceable SuspensionSystem.Components.QuarterCar.QuarterCarModelBase quarterCarModel annotation(
        Placement(visible = true, transformation(origin = {0, -8}, extent = {{-26, -26}, {26, 26}}, rotation = 0)));
      inner Modelica.Mechanics.MultiBody.World world annotation(
        Placement(visible = true, transformation(origin = {-87, -89}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation z(animation = false, r = {0, 0, -1}) annotation(
        Placement(visible = true, transformation(origin = {44, -94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(animation = false, r = {0, 2, 0}) annotation(
        Placement(visible = true, transformation(origin = {82, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(animation = false, r = {0, 0, 0.8}) annotation(
        Placement(visible = true, transformation(origin = {20, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(animation = false,n = {0, -0.8, 0}, s(start = 0.8)) annotation(
        Placement(visible = true, transformation(origin = {-2, 60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Ramp ramp(duration = 20, height = 1.5, offset = 0.5)  annotation(
        Placement(visible = true, transformation(origin = {-60, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      accSquared = quarterCarModel.bodyMass.a_0[2]^2;
      connect(quarterCarModel.zR, roadProfile.zR) annotation(
        Line(points = {{-1, -34}, {-1, -49}}, color = {95, 95, 95}));
      connect(world.frame_b, roadProfile.world_a) annotation(
        Line(points = {{-78, -88}, {-32, -88}, {-32, -73}, {-16, -73}}, color = {95, 95, 95}));
      connect(world.frame_b, z.frame_a) annotation(
        Line(points = {{-78, -88}, {-40, -88}, {-40, -94}, {34, -94}}));
      connect(z.frame_b, fixedTranslation.frame_a) annotation(
        Line(points = {{54, -94}, {82, -94}, {82, 46}}));
      connect(fixedTranslation1.frame_a, fixedTranslation.frame_b) annotation(
        Line(points = {{30, 90}, {82, 90}, {82, 66}}));
      connect(prismatic.frame_b, quarterCarModel.z) annotation(
        Line(points = {{-2, 50}, {-2, 16}}));
      connect(prismatic.frame_a, fixedTranslation1.frame_b) annotation(
        Line(points = {{-2, 70}, {-2, 90}, {10, 90}}));
  connect(ramp.y, roadProfile.RoadRoughness) annotation(
        Line(points = {{-48, -62}, {-16, -62}}, color = {0, 0, 127}));
      annotation(
        Diagram(graphics = {Bitmap(extent = {{-10, -68}, {-10, -68}})}));
    end BaseSuspension;
  end Components;

  package Examples
    model PassiveSuspension
      extends Components.BaseSuspension(redeclare Components.QuarterCar.QuarterCarModelPassive quarterCarModel(d = 1300, unstretchedLen = 1), roadProfile.roadRoughness = 5);
    equation

    annotation(
        experiment(StartTime = 0, StopTime = 30, Tolerance = 1e-6, Interval = 0.002));end PassiveSuspension;

    model ActiveDampingSuspensionADD
    //  Real accSquared ;
      //  Modelica.Blocks.Continuous.Filter bandPassFilter(f_cut = 80, f_min = 0.5, order = 4);
      extends Components.BaseSuspension(redeclare Components.QuarterCar.QuarteCarModelActiveDamping quarterCarModel(dmax = 2500, dmin = 500, unstretchedLen = 1, kB = 10000), roadProfile.roadRoughness = 5);
      SuspensionSystem.Components.Controller.ControllerADD controller(steepness = 10)  annotation(
        Placement(visible = true, transformation(origin = {-64, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ////Modelica.Blocks.Continuous.Filter filter(f_cut = 80, f_min = 0.5, order = 4)  annotation(
    //    Placement(visible = true, transformation(origin = {-62, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
//  accSquared = quarterCarModel.bodyMass.a_0[2]^2;
//  bandPassFilter.u = accSquared;
//  bandPassFilter.y = vertComfortCost;
      connect(quarterCarModel.z_vel, controller.z_vel_in) annotation(
        Line(points = {{26, 10}, {32, 10}, {32, 28}, {-94, 28}, {-94, 2}, {-74, 2}}, color = {0, 0, 127}));
      connect(quarterCarModel.zW_vel, controller.zW_vel_in) annotation(
        Line(points = {{28, -18}, {40, -18}, {40, 34}, {-98, 34}, {-98, -5}, {-74, -5}}, color = {0, 0, 127}));
      connect(controller.outDampingControl, quarterCarModel.inDampingControl) annotation(
        Line(points = {{-54, -3}, {-44, -3}, {-44, -2}, {-26, -2}}, color = {0, 0, 127}));
    annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));end ActiveDampingSuspensionADD;
    
    model ActiveDampingSuspensionSHADD
      parameter Real kSH = 1;
      parameter Real kADD = 1;
    //  Real accSquared ;
      //  Modelica.Blocks.Continuous.Filter bandPassFilter(f_cut = 80, f_min = 0.5, order = 4);
      extends Components.BaseSuspension(redeclare Components.QuarterCar.QuarteCarModelActiveDamping quarterCarModel(dmax = 2500, dmin = 500, unstretchedLen = 1, kB = 10000), roadProfile.roadRoughness = 5);
      SuspensionSystem.Components.Controller.ControllerSHADD controller(steepness = 10, kSH = kSH, kADD = kADD)  annotation(
        Placement(visible = true, transformation(origin = {-64, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    ////Modelica.Blocks.Continuous.Filter filter(f_cut = 80, f_min = 0.5, order = 4)  annotation(
    //    Placement(visible = true, transformation(origin = {-62, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
//  accSquared = quarterCarModel.bodyMass.a_0[2]^2;
//  bandPassFilter.u = accSquared;
//  bandPassFilter.y = vertComfortCost;
      connect(quarterCarModel.z_vel, controller.z_vel_in) annotation(
        Line(points = {{26, 10}, {32, 10}, {32, 28}, {-94, 28}, {-94, 2}, {-74, 2}}, color = {0, 0, 127}));
      connect(quarterCarModel.zW_vel, controller.zW_vel_in) annotation(
        Line(points = {{28, -18}, {40, -18}, {40, 34}, {-98, 34}, {-98, -5}, {-74, -5}}, color = {0, 0, 127}));
      connect(controller.outDampingControl, quarterCarModel.inDampingControl) annotation(
        Line(points = {{-54, -3}, {-44, -3}, {-44, -2}, {-26, -2}}, color = {0, 0, 127}));
    annotation(
        experiment(StartTime = 0, StopTime = 30, Tolerance = 1e-6, Interval = 0.002));end ActiveDampingSuspensionSHADD;
  end Examples;
  annotation(
    uses(Modelica(version = "4.0.0")));
end SuspensionSystem;
