package SuspensionSystem
  import SI = Modelica.Units.SI;
  package Components   class RoadProfile
      parameter Real roadRoughness = 3 "Road height StdDeviation in cm";
      inner Modelica.Mechanics.MultiBody.World world annotation(
        Placement(visible = true, transformation(origin = {-12, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Joints.Prismatic Road(boxHeight = 0.1, n = {0, 1, 0}, useAxisFlange = true) annotation(
        Placement(visible = true, transformation(origin = {38, -16}, extent = {{-30, -30}, {30, 30}}, rotation = 90)));
      Modelica.Mechanics.Translational.Sources.Position position(a(fixed = false), exact = false, v(fixed = false)) annotation(
        Placement(visible = true, transformation(origin = {-22, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b zR annotation(
        Placement(visible = true, transformation(origin = {38, 96}, extent = {{-16, -16}, {16, 16}}, rotation = 90), iconTransformation(origin = {-2, 96}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
      Modelica.Blocks.Continuous.Filter LPF(f_cut = 1, gain = roadRoughness, order = 1) annotation(
        Placement(visible = true, transformation(origin = {-56, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Noise.NormalNoise normalNoise(enableNoise = true, samplePeriod = 0.01, sigma = 0.05, startTime = 0.1, useAutomaticLocalSeed = false, useGlobalSeed = false) annotation(
        Placement(visible = true, transformation(origin = {-88, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(world.frame_b, Road.frame_a) annotation(
        Line(points = {{-2, -90}, {38, -90}, {38, -46}}));
      connect(position.support, Road.support) annotation(
        Line(points = {{-22, -2}, {-22, -28}, {20, -28}}, color = {0, 127, 0}));
      connect(position.flange, Road.axis) annotation(
        Line(points = {{-12, 8}, {20, 8}}, color = {0, 127, 0}));
  connect(zR, Road.frame_b) annotation(
        Line(points = {{38, 96}, {38, 14}}));
      connect(normalNoise.y, LPF.u) annotation(
        Line(points = {{-77, 8}, {-68, 8}}, color = {0, 0, 127}));
      connect(LPF.y, position.s_ref) annotation(
        Line(points = {{-44, 8}, {-34, 8}}, color = {0, 0, 127}));

    end RoadProfile;

    model QuarterCarModel
      parameter SI.Mass mB = 400 "body mass"; 
      parameter SI.Mass mT = 50 "suspended mass (Tyre)";
      parameter Modelica.Units.SI.TranslationalDampingConstant cB = 1300;
      parameter Modelica.Units.SI.TranslationalSpringConstant kB = 20000;
      //parameter Modelica.Units.SI.TranslationalDampingConstant cT;
      parameter Modelica.Units.SI.TranslationalSpringConstant kT = 250000;
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel suspensionSpringDamper(c = kB, d = cB)  annotation(
        Placement(visible = true, transformation(origin = {0, 32}, extent = {{-24, -24}, {24, 24}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a zR annotation(
        Placement(visible = true, transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {-2, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyShape bodyMass(m = mB)  annotation(
        Placement(visible = true, transformation(origin = {0, 86}, extent = {{-12, -12}, {12, 12}}, rotation = 90)));
  SuspensionSystem.Components.Wheel wheel annotation(
        Placement(visible = true, transformation(origin = {1.77636e-15, -50}, extent = {{-18, -18}, {18, 18}}, rotation = 0)));
    equation
      connect(bodyMass.frame_a, suspensionSpringDamper.frame_b) annotation(
        Line(points = {{0, 74}, {0, 56}}));
  connect(wheel.zR, zR) annotation(
        Line(points = {{0, -66}, {0, -100}}, color = {95, 95, 95}));
  connect(wheel.zT, suspensionSpringDamper.frame_a) annotation(
        Line(points = {{0, -34}, {0, 8}}));
    end QuarterCarModel;
    
    model Wheel
      parameter SI.Mass mT = 50 "suspended mass (Tyre)";
      //parameter Modelica.Units.SI.TranslationalDampingConstant cT;
      parameter SI.TranslationalSpringConstant kT = 250000;
      parameter Modelica.Units.SI.Length wr = 0.35 "wheel radius";
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a zR annotation(
        Placement(visible = true, transformation(origin = {0, -98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -94}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b zT annotation(
        Placement(visible = true, transformation(origin = {28, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 94}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.BodyShape WheelMass(animateSphere = false, height = 0.3, length = wr * 0.6, lengthDirection = {0, 0, 1},m = mT, r = {0, wr * 0.65, 0}, r_0(start = {0, wr, 0}), r_CM = {0, wr * 0.65, 0}, r_shape = {0, wr * 0.65, 0}, width = wr * 1.3) annotation(
        Placement(visible = true, transformation(origin = {0, 24}, extent = {{-26, -26}, {26, 26}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic TyreElasticity(animation = true, n = {0, 1, 0}, s(start = wr / 5), useAxisFlange = true)  annotation(
        Placement(visible = true, transformation(origin = {1, -59}, extent = {{-21, -21}, {21, 21}}, rotation = 90)));
  Modelica.Mechanics.Translational.Components.Spring TyreSpring(c = kT, s_rel(fixed = true, start = wr * 0.28), s_rel0 = wr * 0.35)  annotation(
        Placement(visible = true, transformation(origin = {-52, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder clip(diameter = 0.05, innerDiameter = 0.03, length = 0.2, r = {0, 0.05, -0.07})  annotation(
        Placement(visible = true, transformation(origin = {14, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    equation
  connect(TyreElasticity.frame_a, zR) annotation(
        Line(points = {{1, -80}, {0, -80}, {0, -98}}, color = {95, 95, 95}));
  connect(TyreElasticity.frame_b, WheelMass.frame_a) annotation(
        Line(points = {{1, -38}, {1, -9}, {0, -9}, {0, -2}}));
  connect(TyreElasticity.support, TyreSpring.flange_a) annotation(
        Line(points = {{-11.6, -67.4}, {-51.6, -67.4}, {-51.6, -65.4}}, color = {0, 127, 0}));
  connect(TyreSpring.flange_b, TyreElasticity.axis) annotation(
        Line(points = {{-52, -46}, {-50, -46}, {-50, -42}, {-12, -42}}, color = {0, 127, 0}));
  connect(WheelMass.frame_b, clip.frame_b) annotation(
        Line(points = {{0, 50}, {0, 76}, {4, 76}}, color = {95, 95, 95}));
  connect(clip.frame_a, zT) annotation(
        Line(points = {{24, 76}, {28, 76}, {28, 100}}));
    end Wheel;
  end Components;

  package Examples
    model SimpleSuspension
    SuspensionSystem.Components.RoadProfile roadProfile annotation(
        Placement(visible = true, transformation(origin = {1, -81}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  SuspensionSystem.Components.Wheel wheel annotation(
        Placement(visible = true, transformation(origin = {-1, -23}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
    equation
      connect(wheel.zR, roadProfile.zR) annotation(
        Line(points = {{0, -38}, {0, -66}}));
      annotation(
        Diagram(graphics = {Bitmap(extent = {{-10, -68}, {-10, -68}})}));end SimpleSuspension;
  end Examples;
  annotation(
    uses(Modelica(version = "4.0.0")));
end SuspensionSystem;
