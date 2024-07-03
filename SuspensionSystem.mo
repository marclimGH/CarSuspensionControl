package SuspensionSystem
  import SI = Modelica.Units.SI;

  package Components
    class RoadProfile
      parameter Real roadRoughness = 3 "Road height StdDeviation in cm";
      Modelica.Mechanics.MultiBody.Joints.Prismatic Road(boxColor = {100, 100, 100}, boxHeight = 1, boxWidth = 1, n = {0, 1, 0}, s(start = 0.5), useAxisFlange = true) annotation(
        Placement(visible = true, transformation(origin = {38, -16}, extent = {{-30, -30}, {30, 30}}, rotation = 90)));
      Modelica.Mechanics.Translational.Sources.Position position(a(fixed = false), exact = false, v(fixed = false)) annotation(
        Placement(visible = true, transformation(origin = {-22, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b zR annotation(
        Placement(visible = true, transformation(origin = {38, 96}, extent = {{-16, -16}, {16, 16}}, rotation = 90), iconTransformation(origin = {-2, 96}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
      Modelica.Blocks.Continuous.Filter LPF(f_cut = 1, gain = roadRoughness, order = 1) annotation(
        Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Noise.NormalNoise normalNoise(enableNoise = true, mu = 0, samplePeriod = 0.01, sigma = 0.05, startTime = 0.1, useAutomaticLocalSeed = false, useGlobalSeed = false) annotation(
        Placement(visible = true, transformation(origin = {-120, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a world_a annotation(
        Placement(visible = true, transformation(origin = {-98, -72}, extent = {{-16, -16}, {16, 16}}, rotation = 180), iconTransformation(origin = {-100, -66}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation y(r = {0, -roadRoughness / 100 * 3, 0}) annotation(
        Placement(visible = true, transformation(origin = {38, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Math.Add add annotation(
        Placement(visible = true, transformation(origin = {-54, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.RealExpression realExpression(y = roadRoughness / 100 * 3) annotation(
        Placement(visible = true, transformation(origin = {-88, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(position.support, Road.support) annotation(
        Line(points = {{-22, -2}, {-22, -28}, {20, -28}}, color = {0, 127, 0}));
      connect(position.flange, Road.axis) annotation(
        Line(points = {{-12, 8}, {20, 8}}, color = {0, 127, 0}));
      connect(zR, Road.frame_b) annotation(
        Line(points = {{38, 96}, {38, 14}}));
      connect(normalNoise.y, LPF.u) annotation(
        Line(points = {{-109, 10}, {-102, 10}}, color = {0, 0, 127}));
      connect(world_a, y.frame_a) annotation(
        Line(points = {{-98, -72}, {-16, -72}, {-16, -96}, {38, -96}, {38, -86}}));
      connect(y.frame_b, Road.frame_a) annotation(
        Line(points = {{38, -66}, {38, -46}}, color = {95, 95, 95}));
      connect(add.y, position.s_ref) annotation(
        Line(points = {{-43, 6}, {-38, 6}, {-38, 8}, {-34, 8}}, color = {0, 0, 127}));
      connect(LPF.y, add.u1) annotation(
        Line(points = {{-78, 10}, {-72, 10}, {-72, 12}, {-66, 12}}, color = {0, 0, 127}));
      connect(realExpression.y, add.u2) annotation(
        Line(points = {{-76, -24}, {-72, -24}, {-72, 0}, {-66, 0}}, color = {0, 0, 127}));
    end RoadProfile;

    model Wheel
      parameter SI.Mass mT = 50 "suspended mass (Tyre)";
      //parameter Modelica.Units.SI.TranslationalDampingConstant cT;
      parameter SI.TranslationalSpringConstant kT = 250000;
      parameter Modelica.Units.SI.Length wr = 0.35 "wheel radius";
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a zR annotation(
        Placement(visible = true, transformation(origin = {0, -98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -94}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
      Modelica.Mechanics.MultiBody.Parts.BodyShape WheelMass(animateSphere = false, height = 0.3, length = wr * 0.4, lengthDirection = {0, 0, 1}, m = mT, r = {0, wr * 0.65, 0}, r_0(start = {0, wr, 0}), r_CM = {0, wr * 0.65, 0}, r_shape = {0, wr * 0.65, 0}, width = wr * 2) annotation(
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

    partial model BaseSuspension
      SuspensionSystem.Components.RoadProfile roadProfile annotation(
        Placement(visible = true, transformation(origin = {-1, -63}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
      replaceable SuspensionSystem.Components.QuarterCarModel quarterCarModel annotation(
        Placement(visible = true, transformation(origin = {0, -8}, extent = {{-26, -26}, {26, 26}}, rotation = 0)));
      inner Modelica.Mechanics.MultiBody.World world annotation(
        Placement(visible = true, transformation(origin = {-87, -89}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation z(r = {0, 0, -1}) annotation(
        Placement(visible = true, transformation(origin = {44, -94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 2, 0}) annotation(
        Placement(visible = true, transformation(origin = {82, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = {0, 0, 0.8}) annotation(
        Placement(visible = true, transformation(origin = {20, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(n = {0, -0.8, 0}, s(start = 0.8)) annotation(
        Placement(visible = true, transformation(origin = {-2, 60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    equation
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
      annotation(
        Diagram(graphics = {Bitmap(extent = {{-10, -68}, {-10, -68}})}));
    end BaseSuspension;

    model ActiveDamper
      extends Modelica.Mechanics.Translational.Interfaces.PartialCompliantWithRelativeStates;
      parameter SI.TranslationalDampingConstant dmin(start = 500);
      parameter SI.TranslationalDampingConstant dmax(start = 2000);
      SI.TranslationalDampingConstant d(start = dmax);
      SI.Force f(start = 0);
      Modelica.Blocks.Interfaces.BooleanInput dampingSwitch annotation(
        Placement(visible = true, transformation(origin = {-108, 58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {40, 84}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    equation
      if dampingSwitch == true then
        d = dmax;
      else
        d = dmin;
      end if;
      f = d * v_rel;
      annotation(
        Icon(graphics = {Line(points = {{60, -30}, {-60, -30}, {-60, 30}, {60, 30}}, color = {0, 127, 0}), Line(visible = false, points = {{-100, -100}, {-100, -20}, {-14, -20}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Line(points = {{-90, 0}, {100, 0}}, color = {0, 127, 0}), Text(extent = {{-150, -45}, {150, -75}}, textString = "d=%d"), Line(points = {{-60, -30}, {-60, 30}}), Rectangle(lineColor = {0, 127, 0}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-60, 30}, {30, -30}})}));
    end ActiveDamper;

    model ActiveDamper3d
      parameter SI.TranslationalDampingConstant dmax = 2000;
      parameter SI.TranslationalDampingConstant dmin = 500;
      parameter Real d;
      parameter Modelica.Units.SI.Length unstretchedLen = 1;
      SuspensionSystem.Components.ActiveDamper activeDamper(dmax = dmax, dmin = dmin)  annotation(
        Placement(visible = true, transformation(origin = {-37, -3}, extent = {{-19, -19}, {19, 19}}, rotation = 90)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
        Placement(visible = true, transformation(origin = {2, -92}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {98, -4}, extent = {{-16, -16}, {16, 16}}, rotation = 180)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
        Placement(visible = true, transformation(origin = {-2, 92}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {-94, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 180)));
      Modelica.Blocks.Interfaces.BooleanInput dampingSwitch annotation(
        Placement(visible = true, transformation(origin = {-104, 4}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {18, 86}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Forces.LineForceWithMass lineForceWithMass annotation(
        Placement(visible = true, transformation(origin = {39, -1}, extent = {{-33, -33}, {33, 33}}, rotation = 90)));
    equation
      connect(dampingSwitch, activeDamper.dampingSwitch) annotation(
        Line(points = {{-104, 4}, {-52, 4}}, color = {255, 0, 255}));
  connect(frame_b, lineForceWithMass.frame_b) annotation(
        Line(points = {{-2, 92}, {-6, 92}, {-6, 50}, {40, 50}, {40, 32}}));
  connect(frame_a, lineForceWithMass.frame_a) annotation(
        Line(points = {{2, -92}, {2, -58}, {40, -58}, {40, -34}}));
  connect(activeDamper.flange_b, lineForceWithMass.flange_b) annotation(
        Line(points = {{-36, 16}, {6, 16}, {6, 18}}, color = {0, 127, 0}));
  connect(activeDamper.flange_a, lineForceWithMass.flange_a) annotation(
        Line(points = {{-36, -22}, {-15, -22}, {-15, -20}, {6, -20}}, color = {0, 127, 0}));
      annotation(
        Icon(graphics = {Line(points = {{60, -30}, {-60, -30}, {-60, 30}, {60, 30}}, color = {0, 127, 0}), Line(visible = false, points = {{-100, -100}, {-100, -20}, {-14, -20}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Line(points = {{-90, 0}, {100, 0}}, color = {0, 127, 0}), Text(extent = {{-150, -45}, {150, -75}}, textString = "d=%d"), Line(points = {{-60, -30}, {-60, 30}}), Rectangle(lineColor = {0, 127, 0}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-60, 30}, {30, -30}})}));
    end ActiveDamper3d;

        model QuarterCarModel
          parameter SI.Mass mB = 400 "body mass"; 
          parameter SI.Mass mT = 50 "suspended mass (Tyre)";
          parameter Modelica.Units.SI.TranslationalDampingConstant cB = 1300;
          parameter Modelica.Units.SI.TranslationalSpringConstant kB = 20000;
          //parameter Modelica.Units.SI.TranslationalDampingConstant cT;
          parameter Modelica.Units.SI.TranslationalSpringConstant kT = 250000;
      Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel suspensionSpringDamper(c = kB, d = cB, s_unstretched = 1)  annotation(
            Placement(visible = true, transformation(origin = {0, 10}, extent = {{-24, -24}, {24, 24}}, rotation = 90)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a zR annotation(
            Placement(visible = true, transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {-2, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
      SuspensionSystem.Components.Wheel wheel annotation(
            Placement(visible = true, transformation(origin = {1.77636e-15, -50}, extent = {{-18, -18}, {18, 18}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b z annotation(
            Placement(visible = true, transformation(origin = {-2, 130}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {-6, 94}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
      Modelica.Mechanics.MultiBody.Parts.PointMass bodyMass(m = mB, r_0(start = {0, 1.2, 0}), stateSelect = StateSelect.default)  annotation(
            Placement(visible = true, transformation(origin = {0, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        equation
          connect(wheel.zR, zR) annotation(
            Line(points = {{0, -66}, {0, -100}}, color = {95, 95, 95}));
  connect(wheel.zT, suspensionSpringDamper.frame_a) annotation(
        Line(points = {{0, -34}, {0, -14}}));
  connect(bodyMass.frame_a, suspensionSpringDamper.frame_b) annotation(
        Line(points = {{0, 70}, {0, 34}}));
      connect(bodyMass.frame_a, z) annotation(
            Line(points = {{0, 70}, {-2, 70}, {-2, 130}}, color = {95, 95, 95}));
        end QuarterCarModel;

    class QuarteCarModelActiveDamping 
     parameter SI.TranslationalDampingConstant dmax = 2000;
     parameter SI.TranslationalDampingConstant dmin = 500;
     parameter Modelica.Units.SI.Length unstretchedLen = 1;
     extends QuarterCarModelBase(redeclare Components.ActiveDamper3d damper(dmax = cB*2, dmin = cB/2) );
    equation 
      damper.dampingSwitch = true;
    end QuarteCarModelActiveDamping;
    
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
  Modelica.Mechanics.MultiBody.Parts.PointMass bodyMass(m = mB, r_0(start = {0, 1.2, 0}), stateSelect = StateSelect.default) annotation(
        Placement(visible = true, transformation(origin = {0, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Wheel wheel annotation(
        Placement(visible = true, transformation(origin = {1.77636e-15, -50}, extent = {{-18, -18}, {18, 18}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b z annotation(
        Placement(visible = true, transformation(origin = {-2, 130}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {-6, 94}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Forces.Spring spring(c = kB, s_unstretched = unstretchedLen)  annotation(
        Placement(visible = true, transformation(origin = {31, 23}, extent = {{-17, -17}, {17, 17}}, rotation = 90)));
  replaceable Modelica.Mechanics.MultiBody.Forces.Damper damper(d = cB)  annotation(
        Placement(visible = true, transformation(origin = {-44, 20}, extent = {{-18, -18}, {18, 18}}, rotation = 90)));
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
    end QuarterCarModelBase;
  end Components;

  package Examples  model PassiveSuspension
      extends SuspensionSystem.Components.BaseSuspension;
    equation

end PassiveSuspension;

    model ActiveDampingSuspension 
     extends Components.BaseSuspension(redeclare Components.QuarteCarModelActiveDamping quarterCarModel(dmax = 2000, dmin = 500, unstretchedLen = 1));
    end ActiveDampingSuspension;
    
    model PassiveSuspension2
          extends SuspensionSystem.Components.BaseSuspension(redeclare Components.QuarterCarModelBase quarterCarModel);
        equation
    
    end PassiveSuspension2;


  end Examples;
  annotation(
    uses(Modelica(version = "4.0.0")));
end SuspensionSystem;
