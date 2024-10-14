within Tutorial2024.AuxiliaryComponents;

model CraneCrab2
  extends Modelica.Blocks.Icons.Block;
  PlanarMechanics.Parts.Fixed fixed annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  PlanarMechanics.Parts.Body carriage(I = 0.001, m = 1) annotation(
    Placement(transformation(origin = {10, 0}, extent = {{0, -10}, {20, 10}})));
  Modelica.Mechanics.Translational.Sources.Force force(useSupport = false) annotation(
    Placement(transformation(origin = {-80, 0}, extent = {{20, 30}, {40, 50}}, rotation = -0)));
  PlanarMechanics.Joints.Prismatic prismatic(r = {1, 0}, useFlange = true) annotation(
    Placement(transformation(origin = {10, 0}, extent = {{-54, 10}, {-34, -10}})));
  PlanarMechanics.Parts.FixedTranslation rod(r = {0, 0.5}) annotation(
    Placement(transformation(origin = {10, 0}, extent = {{-10, -60}, {10, -40}})));
  PlanarMechanics.Parts.Body pendulum(I = 0.001, m = 0.5) annotation(
    Placement(transformation(origin = {10, 0}, extent = {{20, -60}, {40, -40}})));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation(
    Placement(transformation(origin = {10, 0}, extent = {{-4, -40}, {16, -20}})));
  PlanarMechanics.Joints.Revolute revolute1(useFlange = true, phi(start = 0.1745329251994329, fixed = true), w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {-10, -30}, extent = {{-10, 10}, {10, -10}}, rotation = 270)));
  Modelica.Blocks.Interfaces.RealInput u annotation(
    Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealOutput y annotation(
    Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, -2}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(prismatic.frame_b, carriage.frame_a) annotation(
    Line(points = {{-24, 0}, {10, 0}}, color = {95, 95, 95}));
  connect(revolute1.frame_b, rod.frame_a) annotation(
    Line(points = {{-10, -40}, {-10, -50}, {0, -50}}, color = {95, 95, 95}));
  connect(fixed.frame, prismatic.frame_a) annotation(
    Line(points = {{-60, 0}, {-44, 0}}, color = {95, 95, 95}));
  connect(revolute1.frame_a, carriage.frame_a) annotation(
    Line(points = {{-10, -20}, {-10, 0}, {10, 0}}, color = {95, 95, 95}));
  connect(force.flange, prismatic.flange_a) annotation(
    Line(points = {{-40, 40}, {-34, 40}, {-34, 10}}, color = {0, 127, 0}));
  connect(revolute1.flange_a, angleSensor.flange) annotation(
    Line(points = {{0, -30}, {6, -30}}));
  connect(rod.frame_b, pendulum.frame_a) annotation(
    Line(points = {{20, -50}, {30, -50}}, color = {95, 95, 95}));
  connect(u, force.f) annotation(
    Line(points = {{-120, 0}, {-92, 0}, {-92, 40}, {-62, 40}}, color = {0, 0, 127}));
  connect(angleSensor.phi, y) annotation(
    Line(points = {{28, -30}, {60, -30}, {60, 0}, {110, 0}}, color = {0, 0, 127}));
annotation(
    Icon(graphics = {Line(points = {{-4, -86}, {24, -70}}), Line(points = {{-46, -86}, {-18, -70}}), Line(points = {{20, 58}, {-2, -40}}), Line(points = {{38, -86}, {66, -70}}), Line(points = {{-72, -66}, {78, -66}}), Rectangle(lineColor = {28, 108, 200}, fillPattern = FillPattern.Solid, extent = {{-44, -18}, {46, -62}}), Line(points = {{-84, -86}, {-56, -70}}), Ellipse(lineColor = {28, 108, 200}, fillPattern = FillPattern.Solid, extent = {{8, 78}, {36, 48}})}));
end CraneCrab2;
