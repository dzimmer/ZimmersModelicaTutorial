within Tutorial2024.StepByStep;
model Step3_Pendulum2 "Pendulum"
  extends Modelica.Icons.Example;
  inner PlanarMechanics.PlanarWorld planarWorld annotation (
    Placement(transformation(origin = {240, -20}, extent = {{-160, -60}, {-140, -40}})));
  PlanarMechanics.Parts.Body body(m = 1, I = 0.001) annotation (
    Placement(transformation(origin = {40, 30}, extent = {{20, -10}, {40, 10}})));
  PlanarMechanics.Parts.Fixed fixed annotation (
    Placement(transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  PlanarMechanics.Parts.FixedTranslation rod(r = {2, 0}) annotation (
    Placement(transformation(origin = {10, 30}, extent = {{-10, -10}, {10, 10}})));
  PlanarMechanics.Joints.Revolute revolute(phi(start = 0, fixed = true)) annotation (
    Placement(transformation(origin = {10, 30}, extent = {{-40, -10}, {-20, 10}})));
  PlanarMechanics.Parts.Fixed fixed1 annotation (
    Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  PlanarMechanics.Joints.Revolute revolute1 annotation (
    Placement(transformation(origin = {10, 70}, extent = {{-40, -10}, {-20, 10}})));
  PlanarMechanics.Parts.FixedTranslation fixedTranslation(r = {1, 0}) annotation (
    Placement(transformation(origin = {10, 70}, extent = {{-10, -10}, {10, 10}})));
  PlanarMechanics.Parts.Body body1(I = 0.001, m = 1) annotation (
    Placement(transformation(origin = {12, 70}, extent = {{20, -10}, {40, 10}})));
  PlanarMechanics.Joints.Prismatic prismatic1(r = {0, -1}, useFlange = false) annotation (
    Placement(transformation(origin = {-36, 4}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.Translational.Components.Spring spring(c = 100) annotation (
    Placement(transformation(origin = {-10, -40}, extent = {{-10, -10}, {10, 10}})));
  PlanarMechanics.Joints.Prismatic prismatic(useFlange = true, r = {1, 0}) annotation (
    Placement(transformation(origin = {-10, -10}, extent = {{-10, -10}, {10, 10}})));
  PlanarMechanics.Joints.Revolute revolute2(phi(fixed = false)) annotation (
    Placement(transformation(origin = {0, 30}, extent = {{20, -50}, {40, -30}})));
  PlanarMechanics.Parts.Spring spring1(c_x = 100, c_y = 1e-9, c_phi = 1e-9) annotation (
    Placement(transformation(origin = {20, 60}, extent = {{-40, -160}, {-20, -140}})));
  PlanarMechanics.Parts.Body body2(m = 1, I = 0.001) annotation (
    Placement(transformation(origin = {20, -70}, extent = {{20, -10}, {40, 10}})));
  PlanarMechanics.Parts.Fixed fixed2 annotation (
    Placement(transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  PlanarMechanics.Parts.FixedTranslation rod1(r = {2, 0}) annotation (
    Placement(transformation(origin = {10, 40}, extent = {{-30, -120}, {-10, -100}})));
  PlanarMechanics.Joints.Revolute revolute3(phi(start = 0, fixed = true)) annotation (
    Placement(transformation(origin = {10, 40}, extent = {{-60, -120}, {-40, -100}})));
equation
  connect(rod.frame_b, body.frame_a) annotation (
    Line(points = {{20, 30}, {60, 30}}, color = {95, 95, 95}));
  connect(fixed.frame, revolute.frame_a) annotation (
    Line(points = {{-40, 30}, {-30, 30}}, color = {95, 95, 95}));
  connect(revolute.frame_b, rod.frame_a) annotation (
    Line(points = {{-10, 30}, {0, 30}}, color = {95, 95, 95}));
  connect(fixed1.frame, revolute1.frame_a) annotation (
    Line(points = {{-40, 70}, {-30, 70}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, body1.frame_a) annotation (
    Line(points = {{20, 70}, {32, 70}}, color = {95, 95, 95}));
  connect(revolute1.frame_b, fixedTranslation.frame_a) annotation (
    Line(points = {{-10, 70}, {0, 70}}, color = {95, 95, 95}));
  connect(fixed.frame, prismatic1.frame_a) annotation (
    Line(points = {{-40, 30}, {-36, 30}, {-36, 14}}, color = {95, 95, 95}));
  connect(prismatic1.frame_b, prismatic.frame_a) annotation (
    Line(points = {{-36, -6}, {-35.5, -6}, {-35.5, -4}, {-37, -4}, {-37, -10}, {-20, -10}}, color = {95, 95, 95}));
  connect(prismatic.support, spring.flange_a) annotation (
    Line(points = {{-16, -20}, {-16, -30}, {-20, -30}, {-20, -40}}, color = {0, 127, 0}));
  connect(prismatic.flange_a, spring.flange_b) annotation (
    Line(points = {{-10, -20}, {0, -20}, {0, -40}}, color = {0, 127, 0}));
  connect(prismatic.frame_b, revolute2.frame_a) annotation (
    Line(points = {{0, -10}, {20, -10}}, color = {95, 95, 95}));
  connect(revolute2.frame_b, body.frame_a) annotation (
    Line(points = {{40, -10}, {54, -10}, {54, 30}, {60, 30}}, color = {95, 95, 95}));
  connect(rod1.frame_b, body2.frame_a) annotation (
    Line(points = {{0, -70}, {40, -70}}, color = {95, 95, 95}));
  connect(fixed2.frame, revolute3.frame_a) annotation (
    Line(points = {{-60, -70}, {-50, -70}}, color = {95, 95, 95}));
  connect(revolute3.frame_b, rod1.frame_a) annotation (
    Line(points = {{-30, -70}, {-20, -70}}, color = {95, 95, 95}));
  connect(spring1.frame_b, body2.frame_a) annotation (
    Line(points = {{0, -90}, {20, -90}, {20, -70}, {40, -70}}, color = {95, 95, 95}));
  connect(fixed2.frame, spring1.frame_a) annotation (
    Line(points = {{-60, -70}, {-60, -90}, {-20, -90}}, color = {95, 95, 95}));
  annotation (
    experiment(StopTime = 5, Tolerance = 1e-09, __Dymola_Algorithm = "Dassl"));
end Step3_Pendulum2;
