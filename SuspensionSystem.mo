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
  connect(zR, zR.RoadContactPoint) annotation(
        Line(points = {{38, 96}, {38, 96}}));
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
  Modelica.Mechanics.MultiBody.Forces.Spring tyreSpring(c = kT, s_unstretched = 1)  annotation(
        Placement(visible = true, transformation(origin = {0, -70}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a zR annotation(
        Placement(visible = true, transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {-2, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyShape tyreMass(m = mT)  annotation(
        Placement(visible = true, transformation(origin = {0, -26}, extent = {{-12, -12}, {12, 12}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.BodyShape bodyMass(m = mB)  annotation(
        Placement(visible = true, transformation(origin = {0, 86}, extent = {{-12, -12}, {12, 12}}, rotation = 90)));
    equation
      connect(zR, tyreSpring.frame_a) annotation(
        Line(points = {{0, -100}, {0, -86}}));
  connect(tyreMass.frame_a, tyreSpring.frame_b) annotation(
        Line(points = {{0, -38}, {0, -54}}));
  connect(suspensionSpringDamper.frame_a, tyreMass.frame_b) annotation(
        Line(points = {{0, 8}, {0, -14}}));
  connect(bodyMass.frame_a, suspensionSpringDamper.frame_b) annotation(
        Line(points = {{0, 74}, {0, 56}}));
    end QuarterCarModel;
  end Components;

  package Examples
    model SimpleSuspension
    SuspensionSystem.Components.RoadProfile roadProfile annotation(
        Placement(visible = true, transformation(origin = {-7, -73}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
    equation

    annotation(
        Diagram(graphics = {Bitmap(extent = {{-10, -68}, {-10, -68}})}));end SimpleSuspension;
  end Examples;
  annotation(
    uses(Modelica(version = "4.0.0")));
end SuspensionSystem;
