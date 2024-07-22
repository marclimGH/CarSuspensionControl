package SuspensionSystem
  import SI = Modelica.Units.SI;

  package Components
    model Wheel
      parameter SI.Mass mT = 50 "suspended mass (Tyre)";
      parameter SI.TranslationalSpringConstant kT = 250000;
      parameter Modelica.Units.SI.Length wr = 0.35 "wheel radius";
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a zR;
      Modelica.Mechanics.MultiBody.Parts.BodyShape WheelMass(animateSphere = false, color = {70, 70, 70}, height = 0.3, length = wr*0.4, lengthDirection = {0, 0, 1}, m = mT, r = {0, wr*0.65, 0}, r_0(start = {0, wr, 0}), r_CM = {0, wr*0.65, 0}, r_shape = {0, wr*0.65, 0}, width = wr*2);
      Modelica.Mechanics.MultiBody.Joints.Prismatic TyreElasticity(animation = false, boxColor = {50, 255, 50}, boxHeight = 0.3, n = {0, 1, 0}, s(start = wr/5), useAxisFlange = true);
      Modelica.Mechanics.Translational.Components.Spring TyreSpring(c = kT, s_rel(fixed = true, start = wr*0.28), s_rel0 = wr*0.35);
      Modelica.Mechanics.MultiBody.Parts.BodyCylinder clip(diameter = 0.05, innerDiameter = 0.03, r = {0, 0.1, -0.1});
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b zT;
    equation
      connect(TyreElasticity.frame_a, zR);
      connect(TyreElasticity.frame_b, WheelMass.frame_a);
      connect(TyreElasticity.support, TyreSpring.flange_a);
      connect(TyreSpring.flange_b, TyreElasticity.axis);
      connect(clip.frame_b, zT);
      connect(clip.frame_a, WheelMass.frame_b);
    end Wheel;
  end Components;
end SuspensionSystem;

package ModelicaServices "ModelicaServices (OpenModelica implementation) - Models and functions used in the Modelica Standard Library requiring a tool specific implementation"
  extends Modelica.Icons.Package;

  package Animation "Models and functions for 3-dim. animation"
    extends Modelica.Icons.Package;

    model Shape "Different visual shapes with variable size; all data have to be set as modifiers (see info layer)"
      extends Modelica.Utilities.Internal.PartialModelicaServices.Animation.PartialShape;
    end Shape;

    model Surface "Animation of a moveable, parameterized surface; the surface characteristic is provided by a function"
      extends Modelica.Utilities.Internal.PartialModelicaServices.Animation.PartialSurface;
    end Surface;
  end Animation;

  package Machine "Machine dependent constants"
    extends Modelica.Icons.Package;
    final constant Real eps = 1e-15 "Biggest number such that 1.0 + eps = 1.0";
    final constant Real small = 1e-60 "Smallest number such that small and -small are representable on the machine";
    final constant Real inf = 1e60 "Biggest Real number such that inf and -inf are representable on the machine";
    final constant Integer Integer_inf = OpenModelica.Internal.Architecture.integerMax() "Biggest Integer number such that Integer_inf and -Integer_inf are representable on the machine";
  end Machine;
  annotation(version = "4.0.0", versionDate = "2020-06-04", dateModified = "2020-06-04 11:00:00Z");
end ModelicaServices;

package Modelica "Modelica Standard Library - Version 4.0.0"
  extends Modelica.Icons.Package;

  package Mechanics "Library of 1-dim. and 3-dim. mechanical components (multi-body, rotational, translational)"
    extends Modelica.Icons.Package;
    import Modelica.Units.SI;

    package MultiBody "Library to model 3-dimensional mechanical systems"
      extends Modelica.Icons.Package;
      import Cv = Modelica.Units.Conversions;
      import C = Modelica.Constants;

      model World "World coordinate system + gravity field + default animation definition"
        import Modelica.Mechanics.MultiBody.Types.GravityTypes;
        import Modelica.Mechanics.MultiBody.Types;
        import Modelica.Constants.pi;
        Interfaces.Frame_b frame_b "Coordinate system fixed in the origin of the world frame";
        parameter Boolean enableAnimation = true "= true, if animation of all components is enabled";
        parameter Boolean animateWorld = true "= true, if world coordinate system shall be visualized";
        parameter Boolean animateGravity = true "= true, if gravity field shall be visualized (acceleration vector or field center)";
        parameter Boolean animateGround = false "= true, if ground plane shall be visualized";
        parameter Types.AxisLabel label1 = "x" "Label of horizontal axis in icon";
        parameter Types.AxisLabel label2 = "y" "Label of vertical axis in icon";
        parameter Types.GravityTypes gravityType = GravityTypes.UniformGravity "Type of gravity field" annotation(Evaluate = true);
        parameter SI.Acceleration g = Modelica.Constants.g_n "Constant gravity acceleration";
        parameter Types.Axis n = {0, -1, 0} "Direction of gravity resolved in world frame (gravity = g*n/length(n))" annotation(Evaluate = true);
        parameter Real mu(unit = "m3/s2", min = 0) = 3.986004418e14 "Gravity field constant (default = field constant of earth)";
        parameter Boolean driveTrainMechanics3D = true "= true, if 3-dim. mechanical effects of Parts.Mounting1D/Rotor1D/BevelGear1D shall be taken into account";
        parameter SI.Distance axisLength = nominalLength/2 "Length of world axes arrows";
        parameter SI.Distance axisDiameter = axisLength/defaultFrameDiameterFraction "Diameter of world axes arrows";
        parameter Boolean axisShowLabels = true "= true, if labels shall be shown";
        input Types.Color axisColor_x = Modelica.Mechanics.MultiBody.Types.Defaults.FrameColor "Color of x-arrow";
        input Types.Color axisColor_y = axisColor_x;
        input Types.Color axisColor_z = axisColor_x "Color of z-arrow";
        parameter SI.Position gravityArrowTail[3] = {0, 0, 0} "Position vector from origin of world frame to arrow tail, resolved in world frame";
        parameter SI.Length gravityArrowLength = axisLength/2 "Length of gravity arrow";
        parameter SI.Diameter gravityArrowDiameter = gravityArrowLength/defaultWidthFraction "Diameter of gravity arrow";
        input Types.Color gravityArrowColor = {0, 230, 0} "Color of gravity arrow";
        parameter SI.Diameter gravitySphereDiameter = 12742000 "Diameter of sphere representing gravity center (default = mean diameter of earth)";
        input Types.Color gravitySphereColor = {0, 230, 0} "Color of gravity sphere";
        parameter MultiBody.Types.Axis groundAxis_u = if abs(n[1]) >= 0.99 then {0, 1, 0} else {1, 0, 0} "Vector along 1st axis (called u) of ground plane, resolved in world frame (should be perpendicular to gravity direction)";
        parameter SI.Length groundLength_u = 2 "Length of ground plane along groundAxis_u";
        parameter SI.Length groundLength_v = groundLength_u "Length of ground plane perpendicular to groundAxis_u";
        input Types.Color groundColor = {200, 200, 200} "Color of ground plane";
        parameter SI.Length nominalLength = 1 "Nominal length of multi-body system";
        parameter SI.Length defaultAxisLength = nominalLength/5 "Default for length of a frame axis (but not world frame)";
        parameter SI.Length defaultJointLength = nominalLength/10 "Default for the fixed length of a shape representing a joint";
        parameter SI.Length defaultJointWidth = nominalLength/20 "Default for the fixed width of a shape representing a joint";
        parameter SI.Length defaultForceLength = nominalLength/10 "Default for the fixed length of a shape representing a force (e.g., damper)";
        parameter SI.Length defaultForceWidth = nominalLength/20 "Default for the fixed width of a shape representing a force (e.g., spring, bushing)";
        parameter SI.Length defaultBodyDiameter = nominalLength/9 "Default for diameter of sphere representing the center of mass of a body";
        parameter Real defaultWidthFraction = 20 "Default for shape width as a fraction of shape length (e.g., for Parts.FixedTranslation)";
        parameter SI.Length defaultArrowDiameter = nominalLength/40 "Default for arrow diameter (e.g., of forces, torques, sensors)";
        parameter Real defaultFrameDiameterFraction = 40 "Default for arrow diameter of a coordinate system as a fraction of axis length";
        parameter Real defaultSpecularCoefficient(min = 0) = 0.7 "Default reflection of ambient light (= 0: light is completely absorbed)";
        parameter Real defaultN_to_m(unit = "N/m", min = 0) = 1000 "Default scaling of force arrows (length = force/defaultN_to_m)";
        parameter Real defaultNm_to_m(unit = "N.m/m", min = 0) = 1000 "Default scaling of torque arrows (length = torque/defaultNm_to_m)";
        replaceable function gravityAcceleration = Modelica.Mechanics.MultiBody.Forces.Internal.standardGravityAcceleration(gravityType = gravityType, g = g*Modelica.Math.Vectors.normalizeWithAssert(n), mu = mu) constrainedby Modelica.Mechanics.MultiBody.Interfaces.partialGravityAcceleration;
      protected
        parameter Integer ndim = if enableAnimation and animateWorld then 1 else 0;
        parameter Integer ndim2 = if enableAnimation and animateWorld and axisShowLabels then 1 else 0;
        parameter SI.Length headLength = min(axisLength, axisDiameter*Types.Defaults.FrameHeadLengthFraction);
        parameter SI.Length headWidth = axisDiameter*Types.Defaults.FrameHeadWidthFraction;
        parameter SI.Length lineLength = max(0, axisLength - headLength);
        parameter SI.Length lineWidth = axisDiameter;
        parameter SI.Length scaledLabel = Modelica.Mechanics.MultiBody.Types.Defaults.FrameLabelHeightFraction*axisDiameter;
        parameter SI.Length labelStart = 1.05*axisLength;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape x_arrowLine(shapeType = "cylinder", length = lineLength, width = lineWidth, height = lineWidth, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, color = axisColor_x, specularCoefficient = 0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape x_arrowHead(shapeType = "cone", length = headLength, width = headWidth, height = headWidth, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, color = axisColor_x, r = {lineLength, 0, 0}, specularCoefficient = 0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Internal.Lines x_label(lines = scaledLabel*{[0, 0; 1, 1], [0, 1; 1, 0]}, diameter = axisDiameter, color = axisColor_x, r_lines = {labelStart, 0, 0}, n_x = {1, 0, 0}, n_y = {0, 1, 0}, specularCoefficient = 0) if enableAnimation and animateWorld and axisShowLabels;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape y_arrowLine(shapeType = "cylinder", length = lineLength, width = lineWidth, height = lineWidth, lengthDirection = {0, 1, 0}, widthDirection = {1, 0, 0}, color = axisColor_y, specularCoefficient = 0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape y_arrowHead(shapeType = "cone", length = headLength, width = headWidth, height = headWidth, lengthDirection = {0, 1, 0}, widthDirection = {1, 0, 0}, color = axisColor_y, r = {0, lineLength, 0}, specularCoefficient = 0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Internal.Lines y_label(lines = scaledLabel*{[0, 0; 1, 1.5], [0, 1.5; 0.5, 0.75]}, diameter = axisDiameter, color = axisColor_y, r_lines = {0, labelStart, 0}, n_x = {0, 1, 0}, n_y = {-1, 0, 0}, specularCoefficient = 0) if enableAnimation and animateWorld and axisShowLabels;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape z_arrowLine(shapeType = "cylinder", length = lineLength, width = lineWidth, height = lineWidth, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = axisColor_z, specularCoefficient = 0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape z_arrowHead(shapeType = "cone", length = headLength, width = headWidth, height = headWidth, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = axisColor_z, r = {0, 0, lineLength}, specularCoefficient = 0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Internal.Lines z_label(lines = scaledLabel*{[0, 0; 1, 0], [0, 1; 1, 1], [0, 1; 1, 0]}, diameter = axisDiameter, color = axisColor_z, r_lines = {0, 0, labelStart}, n_x = {0, 0, 1}, n_y = {0, 1, 0}, specularCoefficient = 0) if enableAnimation and animateWorld and axisShowLabels;
        parameter SI.Length gravityHeadLength = min(gravityArrowLength, gravityArrowDiameter*Types.Defaults.ArrowHeadLengthFraction);
        parameter SI.Length gravityHeadWidth = gravityArrowDiameter*Types.Defaults.ArrowHeadWidthFraction;
        parameter SI.Length gravityLineLength = max(0, gravityArrowLength - gravityHeadLength);
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape gravityArrowLine(shapeType = "cylinder", length = gravityLineLength, width = gravityArrowDiameter, height = gravityArrowDiameter, lengthDirection = n, widthDirection = {0, 1, 0}, color = gravityArrowColor, r_shape = gravityArrowTail, specularCoefficient = 0) if enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape gravityArrowHead(shapeType = "cone", length = gravityHeadLength, width = gravityHeadWidth, height = gravityHeadWidth, lengthDirection = n, widthDirection = {0, 1, 0}, color = gravityArrowColor, r_shape = gravityArrowTail + Modelica.Math.Vectors.normalize(n)*gravityLineLength, specularCoefficient = 0) if enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity;
        parameter Integer ndim_pointGravity = if enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity then 1 else 0;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape gravitySphere(shapeType = "sphere", r_shape = {-gravitySphereDiameter/2, 0, 0}, lengthDirection = {1, 0, 0}, length = gravitySphereDiameter, width = gravitySphereDiameter, height = gravitySphereDiameter, color = gravitySphereColor, specularCoefficient = 0) if enableAnimation and animateGravity and gravityType == GravityTypes.PointGravity;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Surface surface(final multiColoredSurface = false, final wireframe = false, final color = groundColor, final specularCoefficient = 0, final transparency = 0, final R = Modelica.Mechanics.MultiBody.Frames.absoluteRotation(Modelica.Mechanics.MultiBody.Frames.from_nxy(n, groundAxis_u), Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {pi/2, pi/2, 0}, {0, 0, 0})), final r_0 = zeros(3), final nu = 2, final nv = 2, redeclare function surfaceCharacteristic = Modelica.Mechanics.MultiBody.Visualizers.Advanced.SurfaceCharacteristics.rectangle(lu = groundLength_u, lv = groundLength_v)) if enableAnimation and animateGround and gravityType == GravityTypes.UniformGravity;
      equation
        Connections.root(frame_b.R);
        assert(Modelica.Math.Vectors.length(n) > 1e-10, "Parameter n of World object is wrong (length(n) > 0 required)");
        frame_b.r_0 = zeros(3);
        frame_b.R = Frames.nullRotation();
        annotation(defaultComponentPrefixes = "inner", missingInnerMessage = "No \"world\" component is defined. A default world
      component with the default gravity field will be used
      (g=9.81 in negative y-axis). If this is not desired,
      drag Modelica.Mechanics.MultiBody.World into the top level of your model.");
      end World;

      package Forces "Components that exert forces and/or torques between frames"
        extends Modelica.Icons.SourcesPackage;

        package Internal "Internal package, should not be used by user"
          extends Modelica.Icons.InternalPackage;

          function standardGravityAcceleration "Standard gravity fields (no/parallel/point field)"
            extends Modelica.Icons.Function;
            extends Modelica.Mechanics.MultiBody.Interfaces.partialGravityAcceleration;
            import Modelica.Mechanics.MultiBody.Types.GravityTypes;
            input GravityTypes gravityType "Type of gravity field";
            input SI.Acceleration g[3] "Constant gravity acceleration, resolved in world frame, if gravityType=UniformGravity";
            input Real mu(unit = "m3/s2") "Field constant of point gravity field, if gravityType=PointGravity";
          algorithm
            gravity := if gravityType == GravityTypes.UniformGravity then g else if gravityType == GravityTypes.PointGravity then -(mu/(r*r))*(r/Modelica.Math.Vectors.length(r)) else zeros(3);
            annotation(Inline = true);
          end standardGravityAcceleration;
        end Internal;
      end Forces;

      package Frames "Functions to transform rotational frame quantities"
        extends Modelica.Icons.Package;

        record Orientation "Orientation object defining rotation from a frame 1 into a frame 2"
          extends Modelica.Icons.Record;
          Real T[3, 3] "Transformation matrix from world frame to local frame";
          SI.AngularVelocity w[3] "Absolute angular velocity of local frame, resolved in local frame";

          encapsulated function equalityConstraint "Return the constraint residues to express that two frames have the same orientation"
            import Modelica;
            import Modelica.Mechanics.MultiBody.Frames;
            extends Modelica.Icons.Function;
            input Frames.Orientation R1 "Orientation object to rotate frame 0 into frame 1";
            input Frames.Orientation R2 "Orientation object to rotate frame 0 into frame 2";
            output Real residue[3] "The rotation angles around x-, y-, and z-axis of frame 1 to rotate frame 1 into frame 2 for a small rotation (should be zero)";
          algorithm
            residue := {Modelica.Math.atan2(cross(R1.T[1, :], R1.T[2, :])*R2.T[2, :], R1.T[1, :]*R2.T[1, :]), Modelica.Math.atan2(-cross(R1.T[1, :], R1.T[2, :])*R2.T[1, :], R1.T[2, :]*R2.T[2, :]), Modelica.Math.atan2(R1.T[2, :]*R2.T[1, :], R1.T[3, :]*R2.T[3, :])};
            annotation(Inline = true);
          end equalityConstraint;
        end Orientation;

        function angularVelocity2 "Return angular velocity resolved in frame 2 from orientation object"
          extends Modelica.Icons.Function;
          input Orientation R "Orientation object to rotate frame 1 into frame 2";
          output SI.AngularVelocity w[3] "Angular velocity of frame 2 with respect to frame 1 resolved in frame 2";
        algorithm
          w := R.w;
          annotation(Inline = true);
        end angularVelocity2;

        function resolve1 "Transform vector from frame 2 to frame 1"
          extends Modelica.Icons.Function;
          input Orientation R "Orientation object to rotate frame 1 into frame 2";
          input Real v2[3] "Vector in frame 2";
          output Real v1[3] "Vector in frame 1";
        algorithm
          v1 := transpose(R.T)*v2;
          annotation(derivative(noDerivative = R) = Internal.resolve1_der, InlineAfterIndexReduction = true);
        end resolve1;

        function resolve2 "Transform vector from frame 1 to frame 2"
          extends Modelica.Icons.Function;
          input Orientation R "Orientation object to rotate frame 1 into frame 2";
          input Real v1[3] "Vector in frame 1";
          output Real v2[3] "Vector in frame 2";
        algorithm
          v2 := R.T*v1;
          annotation(derivative(noDerivative = R) = Internal.resolve2_der, InlineAfterIndexReduction = true);
        end resolve2;

        function resolveDyade1 "Transform second order tensor from frame 2 to frame 1"
          extends Modelica.Icons.Function;
          input Orientation R "Orientation object to rotate frame 1 into frame 2";
          input Real D2[3, 3] "Second order tensor resolved in frame 2";
          output Real D1[3, 3] "Second order tensor resolved in frame 1";
        algorithm
          D1 := transpose(R.T)*D2*R.T;
          annotation(Inline = true);
        end resolveDyade1;

        function nullRotation "Return orientation object that does not rotate a frame"
          extends Modelica.Icons.Function;
          output Orientation R "Orientation object such that frame 1 and frame 2 are identical";
        algorithm
          R := Orientation(T = identity(3), w = zeros(3));
          annotation(Inline = true);
        end nullRotation;

        function absoluteRotation "Return absolute orientation object from another absolute and a relative orientation object"
          extends Modelica.Icons.Function;
          input Orientation R1 "Orientation object to rotate frame 0 into frame 1";
          input Orientation R_rel "Orientation object to rotate frame 1 into frame 2";
          output Orientation R2 "Orientation object to rotate frame 0 into frame 2";
        algorithm
          R2 := Orientation(T = R_rel.T*R1.T, w = resolve2(R_rel, R1.w) + R_rel.w);
          annotation(Inline = true);
        end absoluteRotation;

        function planarRotationAngle "Return angle of a planar rotation, given the rotation axis and the representations of a vector in frame 1 and frame 2"
          extends Modelica.Icons.Function;
          input Real e[3](each final unit = "1") "Normalized axis of rotation to rotate frame 1 around e into frame 2 (must have length=1)";
          input Real v1[3] "A vector v resolved in frame 1 (shall not be parallel to e)";
          input Real v2[3] "Vector v resolved in frame 2, i.e., v2 = resolve2(planarRotation(e,angle),v1)";
          output SI.Angle angle "Rotation angle to rotate frame 1 into frame 2 along axis e in the range: -pi <= angle <= pi";
        algorithm
          angle := Modelica.Math.atan2(-cross(e, v1)*v2, v1*v2 - (e*v1)*(e*v2));
          annotation(Inline = true);
        end planarRotationAngle;

        function axesRotations "Return fixed rotation object to rotate in sequence around fixed angles along 3 axes"
          import TM = Modelica.Mechanics.MultiBody.Frames.TransformationMatrices;
          extends Modelica.Icons.Function;
          input Integer sequence[3](min = {1, 1, 1}, max = {3, 3, 3}) = {1, 2, 3} "Sequence of rotations from frame 1 to frame 2 along axis sequence[i]";
          input SI.Angle angles[3] "Rotation angles around the axes defined in 'sequence'";
          input SI.AngularVelocity der_angles[3] "= der(angles)";
          output Orientation R "Orientation object to rotate frame 1 into frame 2";
        algorithm
          R := Orientation(T = TM.axisRotation(sequence[3], angles[3])*TM.axisRotation(sequence[2], angles[2])*TM.axisRotation(sequence[1], angles[1]), w = Frames.axis(sequence[3])*der_angles[3] + TM.resolve2(TM.axisRotation(sequence[3], angles[3]), Frames.axis(sequence[2])*der_angles[2]) + TM.resolve2(TM.axisRotation(sequence[3], angles[3])*TM.axisRotation(sequence[2], angles[2]), Frames.axis(sequence[1])*der_angles[1]));
          annotation(Inline = true);
        end axesRotations;

        function axesRotationsAngles "Return the 3 angles to rotate in sequence around 3 axes to construct the given orientation object"
          extends Modelica.Icons.Function;
          input Orientation R "Orientation object to rotate frame 1 into frame 2";
          input Integer sequence[3](min = {1, 1, 1}, max = {3, 3, 3}) = {1, 2, 3} "Sequence of rotations from frame 1 to frame 2 along axis sequence[i]";
          input SI.Angle guessAngle1 = 0 "Select angles[1] such that |angles[1] - guessAngle1| is a minimum";
          output SI.Angle angles[3] "Rotation angles around the axes defined in 'sequence' such that R=Frames.axesRotation(sequence,angles); -pi < angles[i] <= pi";
        protected
          Real e1_1[3](each final unit = "1") "First rotation axis, resolved in frame 1";
          Real e2_1a[3](each final unit = "1") "Second rotation axis, resolved in frame 1a";
          Real e3_1[3](each final unit = "1") "Third rotation axis, resolved in frame 1";
          Real e3_2[3](each final unit = "1") "Third rotation axis, resolved in frame 2";
          Real A "Coefficient A in the equation A*cos(angles[1])+B*sin(angles[1]) = 0";
          Real B "Coefficient B in the equation A*cos(angles[1])+B*sin(angles[1]) = 0";
          SI.Angle angle_1a "Solution 1 for angles[1]";
          SI.Angle angle_1b "Solution 2 for angles[1]";
          TransformationMatrices.Orientation T_1a "Orientation object to rotate frame 1 into frame 1a";
        algorithm
          assert(sequence[1] <> sequence[2] and sequence[2] <> sequence[3], "input argument 'sequence[1:3]' is not valid");
          e1_1 := if sequence[1] == 1 then {1, 0, 0} else if sequence[1] == 2 then {0, 1, 0} else {0, 0, 1};
          e2_1a := if sequence[2] == 1 then {1, 0, 0} else if sequence[2] == 2 then {0, 1, 0} else {0, 0, 1};
          e3_1 := R.T[sequence[3], :];
          e3_2 := if sequence[3] == 1 then {1, 0, 0} else if sequence[3] == 2 then {0, 1, 0} else {0, 0, 1};
          A := e2_1a*e3_1;
          B := cross(e1_1, e2_1a)*e3_1;
          if abs(A) <= 1e-12 and abs(B) <= 1e-12 then
            angles[1] := guessAngle1;
          else
            angle_1a := Modelica.Math.atan2(A, -B);
            angle_1b := Modelica.Math.atan2(-A, B);
            angles[1] := if abs(angle_1a - guessAngle1) <= abs(angle_1b - guessAngle1) then angle_1a else angle_1b;
          end if;
          T_1a := TransformationMatrices.planarRotation(e1_1, angles[1]);
          angles[2] := planarRotationAngle(e2_1a, TransformationMatrices.resolve2(T_1a, e3_1), e3_2);
          angles[3] := planarRotationAngle(e3_2, e2_1a, TransformationMatrices.resolve2(R.T, TransformationMatrices.resolve1(T_1a, e2_1a)));
        end axesRotationsAngles;

        function from_nxy "Return fixed orientation object from n_x and n_y vectors"
          extends Modelica.Icons.Function;
          input Real n_x[3](each final unit = "1") "Vector in direction of x-axis of frame 2, resolved in frame 1";
          input Real n_y[3](each final unit = "1") "Vector in direction of y-axis of frame 2, resolved in frame 1";
          output Orientation R "Orientation object to rotate frame 1 into frame 2";
        algorithm
          R := Orientation(T = TransformationMatrices.from_nxy(n_x, n_y), w = zeros(3));
        end from_nxy;

        function from_Q "Return orientation object R from quaternion orientation object Q"
          extends Modelica.Icons.Function;
          input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
          input SI.AngularVelocity w[3] "Angular velocity from frame 2 with respect to frame 1, resolved in frame 2";
          output Orientation R "Orientation object to rotate frame 1 into frame 2";
        algorithm
          R := Orientation([2*(Q[1]*Q[1] + Q[4]*Q[4]) - 1, 2*(Q[1]*Q[2] + Q[3]*Q[4]), 2*(Q[1]*Q[3] - Q[2]*Q[4]); 2*(Q[2]*Q[1] - Q[3]*Q[4]), 2*(Q[2]*Q[2] + Q[4]*Q[4]) - 1, 2*(Q[2]*Q[3] + Q[1]*Q[4]); 2*(Q[3]*Q[1] + Q[2]*Q[4]), 2*(Q[3]*Q[2] - Q[1]*Q[4]), 2*(Q[3]*Q[3] + Q[4]*Q[4]) - 1], w = w);
          annotation(Inline = true);
        end from_Q;

        function to_Q "Return quaternion orientation object Q from orientation object R"
          extends Modelica.Icons.Function;
          input Orientation R "Orientation object to rotate frame 1 into frame 2";
          input Quaternions.Orientation Q_guess = Quaternions.nullRotation() "Guess value for output Q (there are 2 solutions; the one closer to Q_guess is used)";
          output Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
        algorithm
          Q := Quaternions.from_T(R.T, Q_guess);
          annotation(Inline = true);
        end to_Q;

        function axis "Return unit vector for x-, y-, or z-axis"
          extends Modelica.Icons.Function;
          input Integer axis(min = 1, max = 3) "Axis vector to be returned";
          output Real e[3](each final unit = "1") "Unit axis vector";
        algorithm
          e := if axis == 1 then {1, 0, 0} else (if axis == 2 then {0, 1, 0} else {0, 0, 1});
          annotation(Inline = true);
        end axis;

        package Quaternions "Functions to transform rotational frame quantities based on quaternions (also called Euler parameters)"
          extends Modelica.Icons.FunctionsPackage;

          type Orientation "Orientation type defining rotation from a frame 1 into a frame 2 with quaternions {p1,p2,p3,p0}"
            extends Internal.QuaternionBase;

            encapsulated function equalityConstraint "Return the constraint residues to express that two frames have the same quaternion orientation"
              import Modelica;
              import Modelica.Mechanics.MultiBody.Frames.Quaternions;
              extends Modelica.Icons.Function;
              input Quaternions.Orientation Q1 "Quaternions orientation object to rotate frame 0 into frame 1";
              input Quaternions.Orientation Q2 "Quaternions orientation object to rotate frame 0 into frame 2";
              output Real residue[3] "Zero vector if Q1 and Q2 are identical (the first three elements of the relative transformation (is {0,0,0} for the null rotation, guarded by atan2 to make the mirrored solution invalid";
            algorithm
              residue := {Modelica.Math.atan2({Q1[4], Q1[3], -Q1[2], -Q1[1]}*Q2, Q1*Q2), Modelica.Math.atan2({-Q1[3], Q1[4], Q1[1], -Q1[2]}*Q2, Q1*Q2), Modelica.Math.atan2({Q1[2], -Q1[1], Q1[4], -Q1[3]}*Q2, Q1*Q2)};
              annotation(Inline = true);
            end equalityConstraint;
          end Orientation;

          type der_Orientation = Real[4](each unit = "1/s") "First time derivative of Quaternions.Orientation";

          function orientationConstraint "Return residues of orientation constraints (shall be zero)"
            extends Modelica.Icons.Function;
            input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
            output Real residue[1] "Residue constraint (shall be zero)";
          algorithm
            residue := {Q*Q - 1};
            annotation(Inline = true);
          end orientationConstraint;

          function angularVelocity2 "Compute angular velocity resolved in frame 2 from quaternions orientation object and its derivative"
            extends Modelica.Icons.Function;
            input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
            input der_Orientation der_Q "Derivative of Q";
            output SI.AngularVelocity w[3] "Angular velocity of frame 2 with respect to frame 1 resolved in frame 2";
          algorithm
            w := 2*([Q[4], Q[3], -Q[2], -Q[1]; -Q[3], Q[4], Q[1], -Q[2]; Q[2], -Q[1], Q[4], -Q[3]]*der_Q);
            annotation(Inline = true);
          end angularVelocity2;

          function nullRotation "Return quaternion orientation object that does not rotate a frame"
            extends Modelica.Icons.Function;
            output Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
          algorithm
            Q := {0, 0, 0, 1};
            annotation(Inline = true);
          end nullRotation;

          function from_T "Return quaternion orientation object Q from transformation matrix T"
            extends Modelica.Icons.Function;
            input Real T[3, 3] "Transformation matrix to transform vector from frame 1 to frame 2 (v2=T*v1)";
            input Quaternions.Orientation Q_guess = nullRotation() "Guess value for Q (there are 2 solutions; the one close to Q_guess is used";
            output Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2 (Q and -Q have same transformation matrix)";
          protected
            Real paux;
            Real paux4;
            Real c1;
            Real c2;
            Real c3;
            Real c4;
            constant Real p4limit = 0.1;
            constant Real c4limit = 4*p4limit*p4limit;
          algorithm
            c1 := 1 + T[1, 1] - T[2, 2] - T[3, 3];
            c2 := 1 + T[2, 2] - T[1, 1] - T[3, 3];
            c3 := 1 + T[3, 3] - T[1, 1] - T[2, 2];
            c4 := 1 + T[1, 1] + T[2, 2] + T[3, 3];
            if c4 > c4limit or (c4 > c1 and c4 > c2 and c4 > c3) then
              paux := sqrt(c4)/2;
              paux4 := 4*paux;
              Q := {(T[2, 3] - T[3, 2])/paux4, (T[3, 1] - T[1, 3])/paux4, (T[1, 2] - T[2, 1])/paux4, paux};
            elseif c1 > c2 and c1 > c3 and c1 > c4 then
              paux := sqrt(c1)/2;
              paux4 := 4*paux;
              Q := {paux, (T[1, 2] + T[2, 1])/paux4, (T[1, 3] + T[3, 1])/paux4, (T[2, 3] - T[3, 2])/paux4};
            elseif c2 > c1 and c2 > c3 and c2 > c4 then
              paux := sqrt(c2)/2;
              paux4 := 4*paux;
              Q := {(T[1, 2] + T[2, 1])/paux4, paux, (T[2, 3] + T[3, 2])/paux4, (T[3, 1] - T[1, 3])/paux4};
            else
              paux := sqrt(c3)/2;
              paux4 := 4*paux;
              Q := {(T[1, 3] + T[3, 1])/paux4, (T[2, 3] + T[3, 2])/paux4, paux, (T[1, 2] - T[2, 1])/paux4};
            end if;
            if Q*Q_guess < 0 then
              Q := -Q;
            else
            end if;
          end from_T;
        end Quaternions;

        package TransformationMatrices "Functions for transformation matrices"
          extends Modelica.Icons.FunctionsPackage;

          type Orientation "Orientation type defining rotation from a frame 1 into a frame 2 with a transformation matrix"
            extends Internal.TransformationMatrix;

            encapsulated function equalityConstraint "Return the constraint residues to express that two frames have the same orientation"
              import Modelica;
              import Modelica.Mechanics.MultiBody.Frames.TransformationMatrices;
              extends Modelica.Icons.Function;
              input TransformationMatrices.Orientation T1 "Orientation object to rotate frame 0 into frame 1";
              input TransformationMatrices.Orientation T2 "Orientation object to rotate frame 0 into frame 2";
              output Real residue[3] "The rotation angles around x-, y-, and z-axis of frame 1 to rotate frame 1 into frame 2 for a small rotation (should be zero)";
            algorithm
              residue := {cross(T1[1, :], T1[2, :])*T2[2, :], -cross(T1[1, :], T1[2, :])*T2[1, :], T1[2, :]*T2[1, :]};
              annotation(Inline = true);
            end equalityConstraint;
          end Orientation;

          function resolve1 "Transform vector from frame 2 to frame 1"
            extends Modelica.Icons.Function;
            input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
            input Real v2[3] "Vector in frame 2";
            output Real v1[3] "Vector in frame 1";
          algorithm
            v1 := transpose(T)*v2;
            annotation(Inline = true);
          end resolve1;

          function resolve2 "Transform vector from frame 1 to frame 2"
            extends Modelica.Icons.Function;
            input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
            input Real v1[3] "Vector in frame 1";
            output Real v2[3] "Vector in frame 2";
          algorithm
            v2 := T*v1;
            annotation(Inline = true);
          end resolve2;

          function absoluteRotation "Return absolute orientation object from another absolute and a relative orientation object"
            extends Modelica.Icons.Function;
            input TransformationMatrices.Orientation T1 "Orientation object to rotate frame 0 into frame 1";
            input TransformationMatrices.Orientation T_rel "Orientation object to rotate frame 1 into frame 2";
            output TransformationMatrices.Orientation T2 "Orientation object to rotate frame 0 into frame 2";
          algorithm
            T2 := T_rel*T1;
            annotation(Inline = true);
          end absoluteRotation;

          function planarRotation "Return orientation object of a planar rotation"
            import Modelica.Math;
            extends Modelica.Icons.Function;
            input Real e[3](each final unit = "1") "Normalized axis of rotation (must have length=1)";
            input SI.Angle angle "Rotation angle to rotate frame 1 into frame 2 along axis e";
            output TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
          algorithm
            T := outerProduct(e, e) + (identity(3) - outerProduct(e, e))*Math.cos(angle) - skew(e)*Math.sin(angle);
            annotation(Inline = true);
          end planarRotation;

          function axisRotation "Return rotation object to rotate around one frame axis"
            extends Modelica.Icons.Function;
            input Integer axis(min = 1, max = 3) "Rotate around 'axis' of frame 1";
            input SI.Angle angle "Rotation angle to rotate frame 1 into frame 2 along 'axis' of frame 1";
            output TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
          algorithm
            T := if axis == 1 then [1, 0, 0; 0, Modelica.Math.cos(angle), Modelica.Math.sin(angle); 0, -Modelica.Math.sin(angle), Modelica.Math.cos(angle)] else if axis == 2 then [Modelica.Math.cos(angle), 0, -Modelica.Math.sin(angle); 0, 1, 0; Modelica.Math.sin(angle), 0, Modelica.Math.cos(angle)] else [Modelica.Math.cos(angle), Modelica.Math.sin(angle), 0; -Modelica.Math.sin(angle), Modelica.Math.cos(angle), 0; 0, 0, 1];
            annotation(Inline = true);
          end axisRotation;

          function from_nxy "Return orientation object from n_x and n_y vectors"
            extends Modelica.Icons.Function;
            import Modelica.Math.Vectors.length;
            import Modelica.Math.Vectors.normalize;
            input Real n_x[3](each final unit = "1") "Vector in direction of x-axis of frame 2, resolved in frame 1";
            input Real n_y[3](each final unit = "1") "Vector in direction of y-axis of frame 2, resolved in frame 1";
            output TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
          protected
            Real e_x[3](each final unit = "1") = if length(n_x) < 1e-10 then {1, 0, 0} else normalize(n_x);
            Real e_y[3](each final unit = "1") = if length(n_y) < 1e-10 then {0, 1, 0} else normalize(n_y);
            Real n_z_aux[3](each final unit = "1") = cross(e_x, e_y);
            Real n_y_aux[3](each final unit = "1") = if n_z_aux*n_z_aux > 1.0e-6 then e_y else if abs(e_x[1]) > 1.0e-6 then {0, 1, 0} else {1, 0, 0};
            Real e_z_aux[3](each final unit = "1") = cross(e_x, n_y_aux);
            Real e_z[3](each final unit = "1") = normalize(e_z_aux);
          algorithm
            T := {e_x, cross(e_z, e_x), e_z};
          end from_nxy;
        end TransformationMatrices;

        package Internal "Internal definitions that may be removed or changed (do not use)"
          extends Modelica.Icons.InternalPackage;
          type TransformationMatrix = Real[3, 3];
          type QuaternionBase = Real[4];

          function resolve1_der "Derivative of function Frames.resolve1(..)"
            import Modelica.Mechanics.MultiBody.Frames;
            extends Modelica.Icons.Function;
            input Orientation R "Orientation object to rotate frame 1 into frame 2";
            input Real v2[3] "Vector resolved in frame 2";
            input Real v2_der[3] "= der(v2)";
            output Real v1_der[3] "Derivative of vector v resolved in frame 1";
          algorithm
            v1_der := Frames.resolve1(R, v2_der + cross(R.w, v2));
            annotation(Inline = true);
          end resolve1_der;

          function resolve2_der "Derivative of function Frames.resolve2(..)"
            import Modelica.Mechanics.MultiBody.Frames;
            extends Modelica.Icons.Function;
            input Orientation R "Orientation object to rotate frame 1 into frame 2";
            input Real v1[3] "Vector resolved in frame 1";
            input Real v1_der[3] "= der(v1)";
            output Real v2_der[3] "Derivative of vector v resolved in frame 2";
          algorithm
            v2_der := Frames.resolve2(R, v1_der) - cross(R.w, Frames.resolve2(R, v1));
            annotation(Inline = true);
          end resolve2_der;
        end Internal;
      end Frames;

      package Interfaces "Connectors and partial models for 3-dim. mechanical components"
        extends Modelica.Icons.InterfacesPackage;

        connector Frame "Coordinate system fixed to the component with one cut-force and cut-torque (no icon)"
          SI.Position r_0[3] "Position vector from world frame to the connector frame origin, resolved in world frame";
          Frames.Orientation R "Orientation object to rotate the world frame into the connector frame";
          flow SI.Force f[3] "Cut-force resolved in connector frame" annotation(unassignedMessage = "All Forces cannot be uniquely calculated.
        The reason could be that the mechanism contains
        a planar loop or that joints constrain the
        same motion. For planar loops, use for one
        revolute joint per loop the joint
        Joints.RevolutePlanarLoopConstraint instead of
        Joints.Revolute.");
          flow SI.Torque t[3] "Cut-torque resolved in connector frame";
        end Frame;

        connector Frame_a "Coordinate system fixed to the component with one cut-force and cut-torque (filled rectangular icon)"
          extends Frame;
        end Frame_a;

        connector Frame_b "Coordinate system fixed to the component with one cut-force and cut-torque (non-filled rectangular icon)"
          extends Frame;
        end Frame_b;

        partial model PartialElementaryJoint "Base model for elementary joints (has two frames + outer world + assert to guarantee that the joint is connected)"
          Interfaces.Frame_a frame_a "Coordinate system fixed to the joint with one cut-force and cut-torque";
          Interfaces.Frame_b frame_b "Coordinate system fixed to the joint with one cut-force and cut-torque";
        protected
          outer Modelica.Mechanics.MultiBody.World world;
        equation
          Connections.branch(frame_a.R, frame_b.R);
          assert(cardinality(frame_a) > 0, "Connector frame_a of joint object is not connected");
          assert(cardinality(frame_b) > 0, "Connector frame_b of joint object is not connected");
        end PartialElementaryJoint;

        partial function partialGravityAcceleration "Interface for the gravity function used in the World object"
          extends Modelica.Icons.Function;
          input SI.Position r[3] "Position vector from world frame to actual point, resolved in world frame";
          output SI.Acceleration gravity[3] "Gravity acceleration at position r, resolved in world frame";
        end partialGravityAcceleration;

        partial function partialSurfaceCharacteristic "Interface for a function returning surface characteristics"
          extends Modelica.Icons.Function;
          input Integer nu "Number of points in u-Dimension";
          input Integer nv "Number of points in v-Dimension";
          input Boolean multiColoredSurface = false "= true: Color is defined for each surface point";
          output SI.Position X[nu, nv] "[nu,nv] positions of points in x-Direction resolved in surface frame";
          output SI.Position Y[nu, nv] "[nu,nv] positions of points in y-Direction resolved in surface frame";
          output SI.Position Z[nu, nv] "[nu,nv] positions of points in z-Direction resolved in surface frame";
          output Real C[if multiColoredSurface then nu else 0, if multiColoredSurface then nv else 0, 3] "[nu,nv,3] Color array, defining the color for each surface point";
        end partialSurfaceCharacteristic;
      end Interfaces;

      package Joints "Components that constrain the motion between two frames"
        extends Modelica.Icons.Package;

        model Prismatic "Prismatic joint (1 translational degree-of-freedom, 2 potential states, optional axis flange)"
          extends Modelica.Mechanics.MultiBody.Interfaces.PartialElementaryJoint;
          Modelica.Mechanics.Translational.Interfaces.Flange_a axis if useAxisFlange "1-dim. translational flange that drives the joint";
          Modelica.Mechanics.Translational.Interfaces.Flange_b support if useAxisFlange "1-dim. translational flange of the drive support (assumed to be fixed in the world frame, NOT in the joint)";
          parameter Boolean useAxisFlange = false "= true, if axis flange is enabled" annotation(Evaluate = true, HideResult = true);
          parameter Boolean animation = true "= true, if animation shall be enabled";
          parameter Modelica.Mechanics.MultiBody.Types.Axis n = {1, 0, 0} "Axis of translation resolved in frame_a (= same as in frame_b)" annotation(Evaluate = true);
          parameter Types.Axis boxWidthDirection = {0, 1, 0} "Vector in width direction of box, resolved in frame_a" annotation(Evaluate = true);
          parameter SI.Distance boxWidth = world.defaultJointWidth "Width of prismatic joint box";
          parameter SI.Distance boxHeight = boxWidth "Height of prismatic joint box";
          input Types.Color boxColor = Modelica.Mechanics.MultiBody.Types.Defaults.JointColor "Color of prismatic joint box";
          input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)";
          parameter StateSelect stateSelect = StateSelect.prefer "Priority to use distance s and v=der(s) as states";
          final parameter Real e[3](each final unit = "1") = Modelica.Math.Vectors.normalizeWithAssert(n) "Unit vector in direction of prismatic axis n";
          SI.Position s(start = 0, final stateSelect = stateSelect) "Relative distance between frame_a and frame_b" annotation(unassignedMessage = "
        The relative distance s of a prismatic joint cannot be determined.
        Possible reasons:
        - A non-zero mass might be missing on either side of the parts
          connected to the prismatic joint.
        - Too many StateSelect.always are defined and the model
          has less degrees of freedom as specified with this setting
          (remove all StateSelect.always settings).
          ");
          SI.Velocity v(start = 0, final stateSelect = stateSelect) "First derivative of s (relative velocity)";
          SI.Acceleration a(start = 0) "Second derivative of s (relative acceleration)";
          SI.Force f "Actuation force in direction of joint axis";
        protected
          Visualizers.Advanced.Shape box(shapeType = "box", color = boxColor, specularCoefficient = specularCoefficient, length = if noEvent(abs(s) > 1.e-6) then s else 1.e-6, width = boxWidth, height = boxHeight, lengthDirection = e, widthDirection = boxWidthDirection, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
          Translational.Components.Fixed fixed;
          Translational.Interfaces.InternalSupport internalAxis(f = f);
          Translational.Sources.ConstantForce constantForce(f_constant = 0) if not useAxisFlange;
        equation
          v = der(s);
          a = der(v);
          frame_b.r_0 = frame_a.r_0 + Frames.resolve1(frame_a.R, e*s);
          frame_b.R = frame_a.R;
          zeros(3) = frame_a.f + frame_b.f;
          zeros(3) = frame_a.t + frame_b.t + cross(e*s, frame_b.f);
          f = -e*frame_b.f;
          s = internalAxis.s;
          connect(fixed.flange, support);
          connect(internalAxis.flange, axis);
          connect(constantForce.flange, internalAxis.flange);
        end Prismatic;
      end Joints;

      package Parts "Rigid components such as bodies with mass and inertia and massless rods"
        extends Modelica.Icons.Package;

        model FixedTranslation "Fixed translation of frame_b with respect to frame_a"
          import Modelica.Mechanics.MultiBody.Types;
          import Modelica.Units.Conversions.to_unit1;
          Interfaces.Frame_a frame_a "Coordinate system fixed to the component with one cut-force and cut-torque";
          Interfaces.Frame_b frame_b "Coordinate system fixed to the component with one cut-force and cut-torque";
          parameter Boolean animation = true "= true, if animation shall be enabled";
          parameter SI.Position r[3](start = {0, 0, 0}) "Vector from frame_a to frame_b resolved in frame_a";
          parameter Types.ShapeType shapeType = "cylinder" "Type of shape";
          parameter SI.Position r_shape[3] = {0, 0, 0} "Vector from frame_a to shape origin, resolved in frame_a";
          parameter Types.Axis lengthDirection = to_unit1(r - r_shape) "Vector in length direction of shape, resolved in frame_a" annotation(Evaluate = true);
          parameter Types.Axis widthDirection = {0, 1, 0} "Vector in width direction of shape, resolved in frame_a" annotation(Evaluate = true);
          parameter SI.Length length = Modelica.Math.Vectors.length(r - r_shape) "Length of shape";
          parameter SI.Distance width = length/world.defaultWidthFraction "Width of shape";
          parameter SI.Distance height = width "Height of shape";
          parameter Types.ShapeExtra extra = 0.0 "Additional parameter depending on shapeType (see docu of Visualizers.Advanced.Shape)";
          input Types.Color color = Modelica.Mechanics.MultiBody.Types.Defaults.RodColor "Color of shape";
          input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)";
        protected
          outer Modelica.Mechanics.MultiBody.World world;
          Visualizers.Advanced.Shape shape(shapeType = shapeType, color = color, specularCoefficient = specularCoefficient, r_shape = r_shape, lengthDirection = lengthDirection, widthDirection = widthDirection, length = length, width = width, height = height, extra = extra, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
        equation
          Connections.branch(frame_a.R, frame_b.R);
          assert(cardinality(frame_a) > 0 or cardinality(frame_b) > 0, "Neither connector frame_a nor frame_b of FixedTranslation object is connected");
          frame_b.r_0 = frame_a.r_0 + Frames.resolve1(frame_a.R, r);
          frame_b.R = frame_a.R;
          zeros(3) = frame_a.f + frame_b.f;
          zeros(3) = frame_a.t + frame_b.t + cross(r, frame_b.f);
        end FixedTranslation;

        model Body "Rigid body with mass, inertia tensor and one frame connector (12 potential states)"
          import Modelica.Mechanics.MultiBody.Types;
          import Modelica.Mechanics.MultiBody.Frames;
          import Modelica.Units.Conversions.to_unit1;
          Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "Coordinate system fixed at body";
          parameter Boolean animation = true "= true, if animation shall be enabled (show cylinder and sphere)";
          parameter SI.Position r_CM[3](start = {0, 0, 0}) "Vector from frame_a to center of mass, resolved in frame_a";
          parameter SI.Mass m(min = 0, start = 1) "Mass of rigid body";
          parameter SI.Inertia I_11(min = 0) = 0.001 "Element (1,1) of inertia tensor";
          parameter SI.Inertia I_22(min = 0) = 0.001 "Element (2,2) of inertia tensor";
          parameter SI.Inertia I_33(min = 0) = 0.001 "Element (3,3) of inertia tensor";
          parameter SI.Inertia I_21(min = -C.inf) = 0 "Element (2,1) of inertia tensor";
          parameter SI.Inertia I_31(min = -C.inf) = 0 "Element (3,1) of inertia tensor";
          parameter SI.Inertia I_32(min = -C.inf) = 0 "Element (3,2) of inertia tensor";
          SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_a";
          SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))";
          SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))";
          parameter Boolean angles_fixed = false "= true, if angles_start are used as initial values, else as guess values" annotation(Evaluate = true);
          parameter SI.Angle angles_start[3] = {0, 0, 0} "Initial values of angles to rotate world frame around 'sequence_start' axes into frame_a";
          parameter Types.RotationSequence sequence_start = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a at initial time" annotation(Evaluate = true);
          parameter Boolean w_0_fixed = false "= true, if w_0_start are used as initial values, else as guess values" annotation(Evaluate = true);
          parameter SI.AngularVelocity w_0_start[3] = {0, 0, 0} "Initial or guess values of angular velocity of frame_a resolved in world frame";
          parameter Boolean z_0_fixed = false "= true, if z_0_start are used as initial values, else as guess values" annotation(Evaluate = true);
          parameter SI.AngularAcceleration z_0_start[3] = {0, 0, 0} "Initial values of angular acceleration z_0 = der(w_0)";
          parameter SI.Diameter sphereDiameter = world.defaultBodyDiameter "Diameter of sphere";
          input Types.Color sphereColor = Modelica.Mechanics.MultiBody.Types.Defaults.BodyColor "Color of sphere";
          parameter SI.Diameter cylinderDiameter = sphereDiameter/Types.Defaults.BodyCylinderDiameterFraction "Diameter of cylinder";
          input Types.Color cylinderColor = sphereColor "Color of cylinder";
          input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)";
          parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true);
          parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Evaluate = true);
          parameter Types.RotationSequence sequence_angleStates = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a around the 3 angles used as potential states" annotation(Evaluate = true);
          final parameter SI.Inertia I[3, 3] = [I_11, I_21, I_31; I_21, I_22, I_32; I_31, I_32, I_33] "Inertia tensor";
          final parameter Frames.Orientation R_start = Modelica.Mechanics.MultiBody.Frames.axesRotations(sequence_start, angles_start, zeros(3)) "Orientation object from world frame to frame_a at initial time";
          SI.AngularVelocity w_a[3](start = Frames.resolve2(R_start, w_0_start), fixed = fill(w_0_fixed, 3), each stateSelect = if enforceStates then (if useQuaternions then StateSelect.always else StateSelect.never) else StateSelect.avoid) "Absolute angular velocity of frame_a resolved in frame_a";
          SI.AngularAcceleration z_a[3](start = Frames.resolve2(R_start, z_0_start), fixed = fill(z_0_fixed, 3)) "Absolute angular acceleration of frame_a resolved in frame_a";
          SI.Acceleration g_0[3] "Gravity acceleration resolved in world frame";
        protected
          outer Modelica.Mechanics.MultiBody.World world;
          parameter Frames.Quaternions.Orientation Q_start = Frames.to_Q(R_start) "Quaternion orientation object from world frame to frame_a at initial time";
          Frames.Quaternions.Orientation Q(start = Q_start, each stateSelect = if enforceStates then (if useQuaternions then StateSelect.prefer else StateSelect.never) else StateSelect.avoid) "Quaternion orientation object from world frame to frame_a (dummy value, if quaternions are not used as states)";
          parameter SI.Angle phi_start[3] = if sequence_start[1] == sequence_angleStates[1] and sequence_start[2] == sequence_angleStates[2] and sequence_start[3] == sequence_angleStates[3] then angles_start else Frames.axesRotationsAngles(R_start, sequence_angleStates) "Potential angle states at initial time";
          SI.Angle phi[3](start = phi_start, each stateSelect = if enforceStates then (if useQuaternions then StateSelect.never else StateSelect.always) else StateSelect.avoid) "Dummy or 3 angles to rotate world frame into frame_a of body";
          SI.AngularVelocity phi_d[3](each stateSelect = if enforceStates then (if useQuaternions then StateSelect.never else StateSelect.always) else StateSelect.avoid) "= der(phi)";
          SI.AngularAcceleration phi_dd[3] "= der(phi_d)";
          Visualizers.Advanced.Shape cylinder(shapeType = "cylinder", color = cylinderColor, specularCoefficient = specularCoefficient, length = if Modelica.Math.Vectors.length(r_CM) > sphereDiameter/2 then Modelica.Math.Vectors.length(r_CM) - (if cylinderDiameter > 1.1*sphereDiameter then sphereDiameter/2 else 0) else 0, width = cylinderDiameter, height = cylinderDiameter, lengthDirection = to_unit1(r_CM), widthDirection = {0, 1, 0}, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
          Visualizers.Advanced.Shape sphere(shapeType = "sphere", color = sphereColor, specularCoefficient = specularCoefficient, length = sphereDiameter, width = sphereDiameter, height = sphereDiameter, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = r_CM - {1, 0, 0}*sphereDiameter/2, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation and sphereDiameter > 0;
        initial equation
          if angles_fixed then
            if not Connections.isRoot(frame_a.R) then
              zeros(3) = Frames.Orientation.equalityConstraint(frame_a.R, R_start);
            elseif useQuaternions then
              zeros(3) = Frames.Quaternions.Orientation.equalityConstraint(Q, Q_start);
            else
              phi = phi_start;
            end if;
          end if;
        equation
          if enforceStates then
            Connections.root(frame_a.R);
          else
            Connections.potentialRoot(frame_a.R);
          end if;
          r_0 = frame_a.r_0;
          if not Connections.isRoot(frame_a.R) then
            Q = {0, 0, 0, 1};
            phi = zeros(3);
            phi_d = zeros(3);
            phi_dd = zeros(3);
          elseif useQuaternions then
            frame_a.R = Frames.from_Q(Q, Frames.Quaternions.angularVelocity2(Q, der(Q)));
            {0} = Frames.Quaternions.orientationConstraint(Q);
            phi = zeros(3);
            phi_d = zeros(3);
            phi_dd = zeros(3);
          else
            phi_d = der(phi);
            phi_dd = der(phi_d);
            frame_a.R = Frames.axesRotations(sequence_angleStates, phi, phi_d);
            Q = {0, 0, 0, 1};
          end if;
          g_0 = world.gravityAcceleration(frame_a.r_0 + Frames.resolve1(frame_a.R, r_CM));
          v_0 = der(frame_a.r_0);
          a_0 = der(v_0);
          w_a = Frames.angularVelocity2(frame_a.R);
          z_a = der(w_a);
          frame_a.f = m*(Frames.resolve2(frame_a.R, a_0 - g_0) + cross(z_a, r_CM) + cross(w_a, cross(w_a, r_CM)));
          frame_a.t = I*z_a + cross(w_a, I*w_a) + cross(r_CM, frame_a.f);
        end Body;

        model BodyShape "Rigid body with mass, inertia tensor, different shapes for animation, and two frame connectors (12 potential states)"
          import Modelica.Mechanics.MultiBody.Types;
          import Modelica.Units.Conversions.to_unit1;
          Interfaces.Frame_a frame_a "Coordinate system fixed to the component with one cut-force and cut-torque";
          Interfaces.Frame_b frame_b "Coordinate system fixed to the component with one cut-force and cut-torque";
          parameter Boolean animation = true "= true, if animation shall be enabled (show shape between frame_a and frame_b and optionally a sphere at the center of mass)";
          parameter Boolean animateSphere = true "= true, if mass shall be animated as sphere provided animation=true";
          parameter SI.Position r[3](start = {0, 0, 0}) "Vector from frame_a to frame_b resolved in frame_a";
          parameter SI.Position r_CM[3](start = {0, 0, 0}) "Vector from frame_a to center of mass, resolved in frame_a";
          parameter SI.Mass m(min = 0, start = 1) "Mass of rigid body";
          parameter SI.Inertia I_11(min = 0) = 0.001 "Element (1,1) of inertia tensor";
          parameter SI.Inertia I_22(min = 0) = 0.001 "Element (2,2) of inertia tensor";
          parameter SI.Inertia I_33(min = 0) = 0.001 "Element (3,3) of inertia tensor";
          parameter SI.Inertia I_21(min = -C.inf) = 0 "Element (2,1) of inertia tensor";
          parameter SI.Inertia I_31(min = -C.inf) = 0 "Element (3,1) of inertia tensor";
          parameter SI.Inertia I_32(min = -C.inf) = 0 "Element (3,2) of inertia tensor";
          SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_a";
          SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))";
          SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))";
          parameter Boolean angles_fixed = false "= true, if angles_start are used as initial values, else as guess values" annotation(Evaluate = true);
          parameter SI.Angle angles_start[3] = {0, 0, 0} "Initial values of angles to rotate world frame around 'sequence_start' axes into frame_a";
          parameter Types.RotationSequence sequence_start = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a at initial time" annotation(Evaluate = true);
          parameter Boolean w_0_fixed = false "= true, if w_0_start are used as initial values, else as guess values" annotation(Evaluate = true);
          parameter SI.AngularVelocity w_0_start[3] = {0, 0, 0} "Initial or guess values of angular velocity of frame_a resolved in world frame";
          parameter Boolean z_0_fixed = false "= true, if z_0_start are used as initial values, else as guess values" annotation(Evaluate = true);
          parameter SI.AngularAcceleration z_0_start[3] = {0, 0, 0} "Initial values of angular acceleration z_0 = der(w_0)";
          parameter Types.ShapeType shapeType = "cylinder" "Type of shape";
          parameter SI.Position r_shape[3] = {0, 0, 0} "Vector from frame_a to shape origin, resolved in frame_a";
          parameter Types.Axis lengthDirection = to_unit1(r - r_shape) "Vector in length direction of shape, resolved in frame_a" annotation(Evaluate = true);
          parameter Types.Axis widthDirection = {0, 1, 0} "Vector in width direction of shape, resolved in frame_a" annotation(Evaluate = true);
          parameter SI.Length length = Modelica.Math.Vectors.length(r - r_shape) "Length of shape";
          parameter SI.Distance width = length/world.defaultWidthFraction "Width of shape";
          parameter SI.Distance height = width "Height of shape";
          parameter Types.ShapeExtra extra = 0.0 "Additional parameter depending on shapeType (see docu of Visualizers.Advanced.Shape)";
          input Types.Color color = Modelica.Mechanics.MultiBody.Types.Defaults.BodyColor "Color of shape";
          parameter SI.Diameter sphereDiameter = 2*width "Diameter of sphere";
          input Types.Color sphereColor = color "Color of sphere of mass";
          input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)";
          parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true);
          parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Evaluate = true);
          parameter Types.RotationSequence sequence_angleStates = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a around the 3 angles used as potential states" annotation(Evaluate = true);
          FixedTranslation frameTranslation(r = r, animation = false);
          Body body(r_CM = r_CM, m = m, I_11 = I_11, I_22 = I_22, I_33 = I_33, I_21 = I_21, I_31 = I_31, I_32 = I_32, animation = false, sequence_start = sequence_start, angles_fixed = angles_fixed, angles_start = angles_start, w_0_fixed = w_0_fixed, w_0_start = w_0_start, z_0_fixed = z_0_fixed, z_0_start = z_0_start, useQuaternions = useQuaternions, sequence_angleStates = sequence_angleStates, enforceStates = false);
        protected
          outer Modelica.Mechanics.MultiBody.World world;
          Visualizers.Advanced.Shape shape1(shapeType = shapeType, color = color, specularCoefficient = specularCoefficient, length = length, width = width, height = height, lengthDirection = lengthDirection, widthDirection = widthDirection, r_shape = r_shape, extra = extra, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
          Visualizers.Advanced.Shape shape2(shapeType = "sphere", color = sphereColor, specularCoefficient = specularCoefficient, length = sphereDiameter, width = sphereDiameter, height = sphereDiameter, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = r_CM - {1, 0, 0}*sphereDiameter/2, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation and animateSphere;
        equation
          r_0 = frame_a.r_0;
          v_0 = der(r_0);
          a_0 = der(v_0);
          connect(frame_a, frameTranslation.frame_a);
          connect(frame_b, frameTranslation.frame_b);
          connect(frame_a, body.frame_a);
        end BodyShape;

        model BodyCylinder "Rigid body with cylinder shape. Mass and animation properties are computed from cylinder data and density (12 potential states)"
          import Modelica.Units.NonSI;
          import Modelica.Mechanics.MultiBody.Types;
          import Modelica.Math.Vectors.normalizeWithAssert;
          import Modelica.Units.Conversions.to_unit1;
          import Modelica.Constants.pi;
          Interfaces.Frame_a frame_a "Coordinate system fixed to the component with one cut-force and cut-torque";
          Interfaces.Frame_b frame_b "Coordinate system fixed to the component with one cut-force and cut-torque";
          parameter Boolean animation = true "= true, if animation shall be enabled (show cylinder between frame_a and frame_b)";
          parameter SI.Position r[3](start = {0.1, 0, 0}) "Vector from frame_a to frame_b, resolved in frame_a";
          parameter SI.Position r_shape[3] = {0, 0, 0} "Vector from frame_a to cylinder origin, resolved in frame_a";
          parameter Modelica.Mechanics.MultiBody.Types.Axis lengthDirection = to_unit1(r - r_shape) "Vector in length direction of cylinder, resolved in frame_a" annotation(Evaluate = true);
          parameter SI.Length length = Modelica.Math.Vectors.length(r - r_shape) "Length of cylinder";
          parameter SI.Distance diameter = length/world.defaultWidthFraction "Diameter of cylinder";
          parameter SI.Distance innerDiameter = 0 "Inner diameter of cylinder (0 <= innerDiameter <= Diameter)";
          parameter SI.Density density = 7700 "Density of cylinder (e.g., steel: 7700 .. 7900, wood : 400 .. 800)";
          input Modelica.Mechanics.MultiBody.Types.Color color = Modelica.Mechanics.MultiBody.Types.Defaults.BodyColor "Color of cylinder";
          input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)";
          SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_a";
          SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))";
          SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))";
          parameter Boolean angles_fixed = false "= true, if angles_start are used as initial values, else as guess values" annotation(Evaluate = true);
          parameter SI.Angle angles_start[3] = {0, 0, 0} "Initial values of angles to rotate world frame around 'sequence_start' axes into frame_a";
          parameter Types.RotationSequence sequence_start = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a at initial time" annotation(Evaluate = true);
          parameter Boolean w_0_fixed = false "= true, if w_0_start are used as initial values, else as guess values" annotation(Evaluate = true);
          parameter SI.AngularVelocity w_0_start[3] = {0, 0, 0} "Initial or guess values of angular velocity of frame_a resolved in world frame";
          parameter Boolean z_0_fixed = false "= true, if z_0_start are used as initial values, else as guess values" annotation(Evaluate = true);
          parameter SI.AngularAcceleration z_0_start[3] = {0, 0, 0} "Initial values of angular acceleration z_0 = der(w_0)";
          parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true);
          parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Evaluate = true);
          parameter Types.RotationSequence sequence_angleStates = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a around the 3 angles used as potential states" annotation(Evaluate = true);
          final parameter SI.Distance radius = diameter/2 "Radius of cylinder";
          final parameter SI.Distance innerRadius = innerDiameter/2 "Inner-Radius of cylinder";
          final parameter SI.Mass mo(min = 0) = density*pi*length*radius*radius "Mass of cylinder without hole";
          final parameter SI.Mass mi(min = 0) = density*pi*length*innerRadius*innerRadius "Mass of hole of cylinder";
          final parameter SI.Inertia I22 = (mo*(length*length + 3*radius*radius) - mi*(length*length + 3*innerRadius*innerRadius))/12 "Inertia with respect to axis through center of mass, perpendicular to cylinder axis";
          final parameter SI.Mass m(min = 0) = mo - mi "Mass of cylinder";
          final parameter Frames.Orientation R = Frames.from_nxy(r, {0, 1, 0}) "Orientation object from frame_a to frame spanned by cylinder axis and axis perpendicular to cylinder axis";
          final parameter SI.Position r_CM[3] = r_shape + normalizeWithAssert(lengthDirection)*length/2 "Position vector from frame_a to center of mass, resolved in frame_a";
          final parameter SI.Inertia I[3, 3] = Frames.resolveDyade1(R, diagonal({(mo*radius*radius - mi*innerRadius*innerRadius)/2, I22, I22})) "Inertia tensor of cylinder with respect to center of mass, resolved in frame parallel to frame_a";
          Body body(r_CM = r_CM, m = m, I_11 = I[1, 1], I_22 = I[2, 2], I_33 = I[3, 3], I_21 = I[2, 1], I_31 = I[3, 1], I_32 = I[3, 2], animation = false, sequence_start = sequence_start, angles_fixed = angles_fixed, angles_start = angles_start, w_0_fixed = w_0_fixed, w_0_start = w_0_start, z_0_fixed = z_0_fixed, z_0_start = z_0_start, useQuaternions = useQuaternions, sequence_angleStates = sequence_angleStates, enforceStates = false);
          FixedTranslation frameTranslation(r = r, animation = animation, shapeType = "pipecylinder", r_shape = r_shape, lengthDirection = lengthDirection, length = length, width = diameter, height = diameter, extra = innerDiameter/diameter, color = color, specularCoefficient = specularCoefficient, widthDirection = {0, 1, 0});
        protected
          outer Modelica.Mechanics.MultiBody.World world;
        equation
          r_0 = frame_a.r_0;
          v_0 = der(r_0);
          a_0 = der(v_0);
          assert(innerDiameter < diameter, "parameter innerDiameter is greater than parameter diameter");
          connect(frameTranslation.frame_a, frame_a);
          connect(frameTranslation.frame_b, frame_b);
          connect(frame_a, body.frame_a);
        end BodyCylinder;
      end Parts;

      package Visualizers "3-dimensional visual objects used for animation"
        extends Modelica.Icons.Package;

        package Advanced "Visualizers that require basic knowledge about Modelica in order to use them"
          extends Modelica.Icons.Package;

          model Shape "Visualizing an elementary object with variable size; all data have to be set as modifiers (see info layer)"
            extends ModelicaServices.Animation.Shape;
            extends Modelica.Utilities.Internal.PartialModelicaServices.Animation.PartialShape;
          end Shape;

          model Surface "Visualizing a moveable, parameterized surface; the surface characteristic is provided by a function"
            extends Modelica.Mechanics.MultiBody.Icons.Surface;
            extends Modelica.Utilities.Internal.PartialModelicaServices.Animation.PartialSurface;
            extends ModelicaServices.Animation.Surface;
          end Surface;

          package SurfaceCharacteristics "Functions returning surface descriptions"
            extends Modelica.Icons.FunctionsPackage;

            function rectangle "Function defining the surface characteristic of a planar rectangle"
              extends Modelica.Mechanics.MultiBody.Interfaces.partialSurfaceCharacteristic(final multiColoredSurface = false);
              input SI.Distance lu = 1 "Length in direction u";
              input SI.Distance lv = 3 "Length in direction v";
            algorithm
              X[:, :] := lu/2*transpose(fill(linspace(-1, 1, nu), nv));
              Y[:, :] := lv/2*fill(linspace(-1, 1, nv), nu);
              Z[:, :] := fill(0, nu, nv);
            end rectangle;
          end SurfaceCharacteristics;
        end Advanced;

        package Internal "Makeshift visualizers for display of three-dimensional fonts using cylinders"
          extends Modelica.Icons.InternalPackage;

          model Lines "Visualizing a set of lines as cylinders with variable size, e.g., used to display characters (no Frame connector)"
            import Modelica.Mechanics.MultiBody;
            import Modelica.Mechanics.MultiBody.Types;
            import Modelica.Mechanics.MultiBody.Frames;
            import T = Modelica.Mechanics.MultiBody.Frames.TransformationMatrices;
            input Modelica.Mechanics.MultiBody.Frames.Orientation R = Frames.nullRotation() "Orientation object to rotate the world frame into the object frame";
            input SI.Position r[3] = {0, 0, 0} "Position vector from origin of world frame to origin of object frame, resolved in world frame";
            input SI.Position r_lines[3] = {0, 0, 0} "Position vector from origin of object frame to the origin of 'lines' frame, resolved in object frame";
            input Real n_x[3](each final unit = "1") = {1, 0, 0} "Vector in direction of x-axis of 'lines' frame, resolved in object frame";
            input Real n_y[3](each final unit = "1") = {0, 1, 0} "Vector in direction of y-axis of 'lines' frame, resolved in object frame";
            input SI.Position lines[:, 2, 2] = zeros(0, 2, 2) "List of start and end points of cylinders resolved in an x-y frame defined by n_x, n_y, e.g., {[0,0;1,1], [0,1;1,0], [2,0; 3,1]}";
            input SI.Length diameter(min = 0) = 0.05 "Diameter of the cylinders defined by lines";
            input Modelica.Mechanics.MultiBody.Types.Color color = {0, 128, 255} "Color of cylinders";
            input Types.SpecularCoefficient specularCoefficient = 0.7 "Reflection of ambient light (= 0: light is completely absorbed)";
          protected
            parameter Integer n = size(lines, 1) "Number of cylinders";
            T.Orientation R_rel = T.from_nxy(n_x, n_y);
            T.Orientation R_lines = T.absoluteRotation(R.T, R_rel);
            SI.Position r_abs[3] = r + T.resolve1(R.T, r_lines);
            Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape cylinders[n](each shapeType = "cylinder", lengthDirection = {T.resolve1(R_rel, vector([lines[i, 2, :] - lines[i, 1, :]; 0])) for i in 1:n}, length = {Modelica.Math.Vectors.length(lines[i, 2, :] - lines[i, 1, :]) for i in 1:n}, r = {r_abs + T.resolve1(R_lines, vector([lines[i, 1, :]; 0])) for i in 1:n}, each width = diameter, each height = diameter, each widthDirection = {0, 1, 0}, each color = color, each R = R, each specularCoefficient = specularCoefficient);
          end Lines;
        end Internal;
      end Visualizers;

      package Types "Constants and types with choices, especially to build menus"
        extends Modelica.Icons.TypesPackage;
        type Axis = Modelica.Icons.TypeReal[3](each final unit = "1") "Axis vector with choices" annotation(Evaluate = true);
        type AxisLabel = Modelica.Icons.TypeString "Label of axis with choices";
        type RotationSequence = Modelica.Icons.TypeInteger[3](min = {1, 1, 1}, max = {3, 3, 3}) "Sequence of planar frame rotations with choices" annotation(Evaluate = true);
        type Color = Modelica.Icons.TypeInteger[3](each min = 0, each max = 255) "RGB representation of color";
        type SpecularCoefficient = Modelica.Icons.TypeReal(min = 0) "Reflection of ambient light (= 0: light is completely absorbed)";
        type ShapeType = Modelica.Icons.TypeString "Type of shape (box, sphere, cylinder, pipecylinder, cone, pipe, beam, gearwheel, spring, <external shape>)";
        type ShapeExtra = Modelica.Icons.TypeReal "Type of the additional data that can be defined for an elementary ShapeType";
        type GravityTypes = enumeration(NoGravity "No gravity field", UniformGravity "Uniform gravity field", PointGravity "Point gravity field") "Enumeration defining the type of the gravity field";

        package Defaults "Default settings of the MultiBody library via constants"
          extends Modelica.Icons.Package;
          constant Types.Color BodyColor = {0, 128, 255} "Default color for body shapes that have mass (light blue)";
          constant Types.Color RodColor = {155, 155, 155} "Default color for massless rod shapes (grey)";
          constant Types.Color JointColor = {255, 0, 0} "Default color for elementary joints (red)";
          constant Types.Color FrameColor = {0, 0, 0} "Default color for frame axes and labels (black)";
          constant Real FrameHeadLengthFraction = 5.0 "Frame arrow head length / arrow diameter";
          constant Real FrameHeadWidthFraction = 3.0 "Frame arrow head width / arrow diameter";
          constant Real FrameLabelHeightFraction = 3.0 "Height of frame label / arrow diameter";
          constant Real ArrowHeadLengthFraction = 4.0 "Arrow head length / arrow diameter";
          constant Real ArrowHeadWidthFraction = 3.0 "Arrow head width / arrow diameter";
          constant Real BodyCylinderDiameterFraction = 3 "Default for body cylinder diameter as a fraction of body sphere diameter";
        end Defaults;
      end Types;

      package Icons "Icons for MultiBody package"
        extends Modelica.Icons.IconsPackage;

        model Surface "Surface icon" end Surface;
      end Icons;
    end MultiBody;

    package Translational "Library to model 1-dimensional, translational mechanical systems"
      extends Modelica.Icons.Package;

      package Components "Components for 1D translational mechanical drive trains"
        extends Modelica.Icons.Package;

        model Fixed "Fixed flange"
          parameter SI.Position s0 = 0 "Fixed offset position of housing";
          Interfaces.Flange_b flange;
        equation
          flange.s = s0;
        end Fixed;

        model Spring "Linear 1D translational spring"
          extends Translational.Interfaces.PartialCompliant;
          parameter SI.TranslationalSpringConstant c(final min = 0, start = 1) "Spring constant";
          parameter SI.Distance s_rel0 = 0 "Unstretched spring length";
        equation
          f = c*(s_rel - s_rel0);
        end Spring;
      end Components;

      package Sources "Sources to drive 1D translational mechanical components"
        extends Modelica.Icons.SourcesPackage;

        model ConstantForce "Constant force, not dependent on speed"
          extends Modelica.Mechanics.Translational.Interfaces.PartialForce;
          parameter SI.Force f_constant "Nominal force (if negative, force is acting as load in positive direction of motion)";
        equation
          f = -f_constant;
        end ConstantForce;
      end Sources;

      package Interfaces "Interfaces for 1-dim. translational mechanical components"
        extends Modelica.Icons.InterfacesPackage;

        connector Flange "One-dimensional translational flange"
          SI.Position s "Absolute position of flange";
          flow SI.Force f "Cut force directed into flange";
        end Flange;

        connector Flange_a "One-dimensional translational flange (left, flange axis directed INTO cut plane)"
          extends Flange;
        end Flange_a;

        connector Flange_b "One-dimensional translational flange (right, flange axis directed OUT OF cut plane)"
          extends Flange;
        end Flange_b;

        connector Support "Support/housing flange of a one-dimensional translational component"
          extends Flange;
        end Support;

        model InternalSupport "Adapter model to utilize conditional support connector"
          input SI.Force f "External support force (must be computed via force balance in model where InternalSupport is used; = flange.f)";
          SI.Position s "External support position (= flange.s)";
          Flange_a flange "Internal support flange (must be connected to the conditional support connector for useSupport=true and to conditional fixed model for useSupport=false)";
        equation
          flange.f = f;
          flange.s = s;
        end InternalSupport;

        partial model PartialTwoFlanges "Component with two translational 1D flanges"
          Flange_a flange_a "(left) driving flange (flange axis directed into cut plane, e. g. from left to right)";
          Flange_b flange_b "(right) driven flange (flange axis directed out of cut plane)";
        end PartialTwoFlanges;

        partial model PartialCompliant "Compliant connection of two translational 1D flanges"
          extends Translational.Interfaces.PartialTwoFlanges;
          SI.Position s_rel(start = 0) "Relative distance (= flange_b.s - flange_a.s)";
          SI.Force f "Force between flanges (positive in direction of flange axis R)";
        equation
          s_rel = flange_b.s - flange_a.s;
          flange_b.f = f;
          flange_a.f = -f;
        end PartialCompliant;

        partial model PartialElementaryOneFlangeAndSupport2 "Partial model for a component with one translational 1-dim. shaft flange and a support used for textual modeling, i.e., for elementary models"
          parameter Boolean useSupport = false "= true, if support flange enabled, otherwise implicitly grounded" annotation(Evaluate = true, HideResult = true);
          SI.Length s "Distance between flange and support (= flange.s - support.s)";
          Flange_b flange "Flange of component";
          Support support(s = s_support, f = -flange.f) if useSupport "Support/housing of component";
        protected
          SI.Length s_support "Absolute position of support flange";
        equation
          s = flange.s - s_support;
          if not useSupport then
            s_support = 0;
          end if;
        end PartialElementaryOneFlangeAndSupport2;

        partial model PartialForce "Partial model of a force acting at the flange (accelerates the flange)"
          extends PartialElementaryOneFlangeAndSupport2;
          SI.Force f "Accelerating force acting at flange (= flange.f)";
        equation
          f = flange.f;
        end PartialForce;
      end Interfaces;
    end Translational;
  end Mechanics;

  package Math "Library of mathematical functions (e.g., sin, cos) and of functions operating on vectors and matrices"
    extends Modelica.Icons.Package;

    package Vectors "Library of functions operating on vectors"
      extends Modelica.Icons.Package;

      function length "Return length of a vector (better as norm(), if further symbolic processing is performed)"
        extends Modelica.Icons.Function;
        input Real v[:] "Real vector";
        output Real result "Length of vector v";
      algorithm
        result := sqrt(v*v);
        annotation(Inline = true);
      end length;

      function normalize "Return normalized vector such that length = 1 and prevent zero-division for zero vector"
        extends Modelica.Icons.Function;
        input Real v[:] "Real vector";
        input Real eps(min = 0.0) = 100*Modelica.Constants.eps "if |v| < eps then result = v/eps";
        output Real result[size(v, 1)] "Input vector v normalized to length=1";
      algorithm
        result := smooth(0, if length(v) >= eps then v/length(v) else v/eps);
        annotation(Inline = true);
      end normalize;

      function normalizeWithAssert "Return normalized vector such that length = 1 (trigger an assert for zero vector)"
        import Modelica.Math.Vectors.length;
        extends Modelica.Icons.Function;
        input Real v[:] "Real vector";
        output Real result[size(v, 1)] "Input vector v normalized to length=1";
      algorithm
        assert(length(v) > 0.0, "Vector v={0,0,0} shall be normalized (= v/sqrt(v*v)), but this results in a division by zero.\nProvide a non-zero vector!");
        result := v/length(v);
        annotation(Inline = true);
      end normalizeWithAssert;
    end Vectors;

    package Icons "Icons for Math"
      extends Modelica.Icons.IconsPackage;

      partial function AxisLeft "Basic icon for mathematical function with y-axis on left side" end AxisLeft;

      partial function AxisCenter "Basic icon for mathematical function with y-axis in the center" end AxisCenter;
    end Icons;

    function sin "Sine"
      extends Modelica.Math.Icons.AxisLeft;
      input Modelica.Units.SI.Angle u "Independent variable";
      output Real y "Dependent variable y=sin(u)";
      external "builtin" y = sin(u);
    end sin;

    function cos "Cosine"
      extends Modelica.Math.Icons.AxisLeft;
      input Modelica.Units.SI.Angle u "Independent variable";
      output Real y "Dependent variable y=cos(u)";
      external "builtin" y = cos(u);
    end cos;

    function asin "Inverse sine (-1 <= u <= 1)"
      extends Modelica.Math.Icons.AxisCenter;
      input Real u "Independent variable";
      output Modelica.Units.SI.Angle y "Dependent variable y=asin(u)";
      external "builtin" y = asin(u);
    end asin;

    function atan2 "Four quadrant inverse tangent"
      extends Modelica.Math.Icons.AxisCenter;
      input Real u1 "First independent variable";
      input Real u2 "Second independent variable";
      output Modelica.Units.SI.Angle y "Dependent variable y=atan2(u1, u2)=atan(u1/u2)";
      external "builtin" y = atan2(u1, u2);
    end atan2;

    function exp "Exponential, base e"
      extends Modelica.Math.Icons.AxisCenter;
      input Real u "Independent variable";
      output Real y "Dependent variable y=exp(u)";
      external "builtin" y = exp(u);
    end exp;
  end Math;

  package Utilities "Library of utility functions dedicated to scripting (operating on files, streams, strings, system)"
    extends Modelica.Icons.UtilitiesPackage;

    package Internal "Internal components that a user should usually not directly utilize"
      extends Modelica.Icons.InternalPackage;
      import Modelica.Units.SI;

      package PartialModelicaServices "Interfaces of components requiring a tool specific implementation"
        extends Modelica.Icons.InternalPackage;

        package Animation "Models and functions for 3-dim. animation"
          extends Modelica.Icons.Package;

          partial model PartialShape "Interface for 3D animation of elementary shapes"
            import Modelica.Mechanics.MultiBody.Frames;
            import Modelica.Mechanics.MultiBody.Types;
            parameter Types.ShapeType shapeType = "box" "Type of shape (box, sphere, cylinder, pipecylinder, cone, pipe, beam, gearwheel, spring, <external shape>)";
            input Frames.Orientation R = Frames.nullRotation() "Orientation object to rotate the world frame into the object frame";
            input SI.Position r[3] = {0, 0, 0} "Position vector from origin of world frame to origin of object frame, resolved in world frame";
            input SI.Position r_shape[3] = {0, 0, 0} "Position vector from origin of object frame to shape origin, resolved in object frame";
            input Real lengthDirection[3](each final unit = "1") = {1, 0, 0} "Vector in length direction, resolved in object frame";
            input Real widthDirection[3](each final unit = "1") = {0, 1, 0} "Vector in width direction, resolved in object frame";
            input SI.Length length = 0 "Length of visual object";
            input SI.Length width = 0 "Width of visual object";
            input SI.Length height = 0 "Height of visual object";
            input Types.ShapeExtra extra = 0.0 "Additional size data for some of the shape types";
            input Real color[3] = {255, 0, 0} "Color of shape";
            input Types.SpecularCoefficient specularCoefficient = 0.7 "Reflection of ambient light (= 0: light is completely absorbed)";
          end PartialShape;

          partial model PartialSurface "Interface for 3D animation of surfaces"
            import Modelica.Mechanics.MultiBody.Frames;
            import Modelica.Mechanics.MultiBody.Types;
            input Frames.Orientation R = Frames.nullRotation() "Orientation object to rotate the world frame into the surface frame";
            input SI.Position r_0[3] = {0, 0, 0} "Position vector from origin of world frame to origin of surface frame, resolved in world frame";
            parameter Integer nu = 2 "Number of points in u-Dimension";
            parameter Integer nv = 2 "Number of points in v-Dimension";
            replaceable function surfaceCharacteristic = Modelica.Mechanics.MultiBody.Interfaces.partialSurfaceCharacteristic "Function defining the surface characteristic" annotation(choicesAllMatching = true);
            parameter Boolean wireframe = false "= true: 3D model will be displayed without faces";
            parameter Boolean multiColoredSurface = false "= true: Color is defined for each surface point";
            input Real color[3] = {255, 0, 0} "Color of surface";
            input Types.SpecularCoefficient specularCoefficient = 0.7 "Reflection of ambient light (= 0: light is completely absorbed)";
            input Real transparency = 0 "Transparency of shape: 0 (= opaque) ... 1 (= fully transparent)";
          end PartialSurface;
        end Animation;
      end PartialModelicaServices;
    end Internal;
  end Utilities;

  package Constants "Library of mathematical constants and constants of nature (e.g., pi, eps, R, sigma)"
    extends Modelica.Icons.Package;
    import Modelica.Units.SI;
    import Modelica.Units.NonSI;
    final constant Real pi = 2*Modelica.Math.asin(1.0);
    final constant Real eps = ModelicaServices.Machine.eps "Biggest number such that 1.0 + eps = 1.0";
    final constant Real inf = ModelicaServices.Machine.inf "Biggest Real number such that inf and -inf are representable on the machine";
    final constant SI.Velocity c = 299792458 "Speed of light in vacuum";
    final constant SI.Acceleration g_n = 9.80665 "Standard acceleration of gravity on earth";
    final constant SI.ElectricCharge q = 1.602176634e-19 "Elementary charge";
    final constant Real h(final unit = "J.s") = 6.62607015e-34 "Planck constant";
    final constant Real k(final unit = "J/K") = 1.380649e-23 "Boltzmann constant";
    final constant Real N_A(final unit = "1/mol") = 6.02214076e23 "Avogadro constant";
    final constant Real mu_0(final unit = "N/A2") = 4*pi*1.00000000055e-7 "Magnetic constant";
  end Constants;

  package Icons "Library of icons"
    extends Icons.Package;

    partial package Package "Icon for standard packages" end Package;

    partial package InterfacesPackage "Icon for packages containing interfaces"
      extends Modelica.Icons.Package;
    end InterfacesPackage;

    partial package SourcesPackage "Icon for packages containing sources"
      extends Modelica.Icons.Package;
    end SourcesPackage;

    partial package UtilitiesPackage "Icon for utility packages"
      extends Modelica.Icons.Package;
    end UtilitiesPackage;

    partial package TypesPackage "Icon for packages containing type definitions"
      extends Modelica.Icons.Package;
    end TypesPackage;

    partial package FunctionsPackage "Icon for packages containing functions"
      extends Modelica.Icons.Package;
    end FunctionsPackage;

    partial package IconsPackage "Icon for packages containing icons"
      extends Modelica.Icons.Package;
    end IconsPackage;

    partial package InternalPackage "Icon for an internal package (indicating that the package should not be directly utilized by user)" end InternalPackage;

    partial function Function "Icon for functions" end Function;

    partial record Record "Icon for records" end Record;

    type TypeReal "Icon for Real types"
      extends Real;
    end TypeReal;

    type TypeInteger "Icon for Integer types"
      extends Integer;
    end TypeInteger;

    type TypeString "Icon for String types"
      extends String;
    end TypeString;
  end Icons;

  package Units "Library of type and unit definitions"
    extends Modelica.Icons.Package;

    package SI "Library of SI unit definitions"
      extends Modelica.Icons.Package;
      type Angle = Real(final quantity = "Angle", final unit = "rad", displayUnit = "deg");
      type Length = Real(final quantity = "Length", final unit = "m");
      type Position = Length;
      type Distance = Length(min = 0);
      type Diameter = Length(min = 0);
      type AngularVelocity = Real(final quantity = "AngularVelocity", final unit = "rad/s");
      type AngularAcceleration = Real(final quantity = "AngularAcceleration", final unit = "rad/s2");
      type Velocity = Real(final quantity = "Velocity", final unit = "m/s");
      type Acceleration = Real(final quantity = "Acceleration", final unit = "m/s2");
      type Mass = Real(quantity = "Mass", final unit = "kg", min = 0);
      type Density = Real(final quantity = "Density", final unit = "kg/m3", displayUnit = "g/cm3", min = 0.0);
      type MomentOfInertia = Real(final quantity = "MomentOfInertia", final unit = "kg.m2");
      type Inertia = MomentOfInertia;
      type Force = Real(final quantity = "Force", final unit = "N");
      type TranslationalSpringConstant = Real(final quantity = "TranslationalSpringConstant", final unit = "N/m");
      type Torque = Real(final quantity = "Torque", final unit = "N.m");
      type ElectricCharge = Real(final quantity = "ElectricCharge", final unit = "C");
      type FaradayConstant = Real(final quantity = "FaradayConstant", final unit = "C/mol");
    end SI;

    package NonSI "Type definitions of non SI and other units"
      extends Modelica.Icons.Package;
      type Temperature_degC = Real(final quantity = "ThermodynamicTemperature", final unit = "degC") "Absolute temperature in degree Celsius (for relative temperature use Modelica.Units.SI.TemperatureDifference)" annotation(absoluteValue = true);
    end NonSI;

    package Conversions "Conversion functions to/from non SI units and type definitions of non SI units"
      extends Modelica.Icons.Package;

      function to_unit1 "Change the unit of a Real number to unit=\"1\""
        extends Modelica.Units.Icons.Conversion;
        input Real r "Real number";
        output Real result(unit = "1") "Real number r with unit=\"1\"";
      algorithm
        result := r;
        annotation(Inline = true);
      end to_unit1;
    end Conversions;

    package Icons "Icons for Units"
      extends Modelica.Icons.IconsPackage;

      partial function Conversion "Base icon for conversion functions" end Conversion;
    end Icons;
  end Units;
  annotation(version = "4.0.0", versionDate = "2020-06-04", dateModified = "2020-06-04 11:00:00Z");
end Modelica;

model Wheel_total
  extends SuspensionSystem.Components.Wheel;
end Wheel_total;
