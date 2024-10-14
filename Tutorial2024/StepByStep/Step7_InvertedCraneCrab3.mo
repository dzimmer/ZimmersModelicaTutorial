within Tutorial2024.StepByStep;
model Step7_InvertedCraneCrab3
  extends Modelica.Icons.Example;
  inner Modelica.Mechanics.MultiBody.World world annotation (
    Placement(transformation(extent = {{-88, -80}, {-68, -60}})));
  Modelica.Blocks.Continuous.SecondOrder secondOrder(w = 10, D = 1) annotation (
    Placement(transformation(extent = {{-80, -10}, {-60, 10}})));
  Modelica.Blocks.Sources.RealExpression angle(y = 0) annotation (
    Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Modelica.Blocks.Math.InverseBlockConstraints inverseBlockConstraints annotation (
    Placement(transformation(origin = {-8, 0}, extent = {{-12, -10}, {28, 10}})));
  Modelica.Blocks.Math.UnitConversions.From_deg from_deg annotation (
    Placement(transformation(extent = {{-50, -10}, {-30, 10}})));
  AuxiliaryComponents.CraneCrab2 craneCrab1 annotation (
    Placement(transformation( extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  AuxiliaryComponents.CraneCrab2 craneCrab annotation (
    Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(secondOrder.y, from_deg.u) annotation (
    Line(points = {{-59, 0}, {-52, 0}}, color = {0, 0, 127}));
  connect(from_deg.y, inverseBlockConstraints.u1) annotation (
    Line(points = {{-29, 0}, {-22, 0}}, color = {0, 0, 127}));
  connect(inverseBlockConstraints.u2, craneCrab1.y) annotation (
    Line(points = {{-16, 0}, {-11, 0}}, color = {0, 0, 127}));
  connect(inverseBlockConstraints.y2, craneCrab1.u) annotation (
    Line(points = {{17, 0}, {12, 0}}, color = {0, 0, 127}));
  connect(inverseBlockConstraints.y1, craneCrab.u) annotation (
    Line(points = {{21, 0}, {38, 0}}, color = {0, 0, 127}));
  connect(angle.y, secondOrder.u) annotation (
    Line(points = {{-88, 0}, {-82, 0}}, color = {0, 0, 127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio = false)),
    Diagram(coordinateSystem(preserveAspectRatio = false)),
    experiment(__Dymola_NumberOfIntervals = 5000, __Dymola_Algorithm = "Dassl"),
    Documentation(info = "<html>
<p>
Example from Dirk's lecture. It does not make sense tho.
</p>
</html>"));
end Step7_InvertedCraneCrab3;
