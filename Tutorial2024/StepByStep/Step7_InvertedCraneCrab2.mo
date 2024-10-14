within Tutorial2024.StepByStep;
model Step7_InvertedCraneCrab2
  extends Modelica.Icons.Example;
  inner Modelica.Mechanics.MultiBody.World world annotation (Placement(transformation(extent={{-88,-80},{-68,-60}})));
  Modelica.Blocks.Continuous.SecondOrder secondOrder(w=10, D=1)
                                                          annotation (Placement(transformation(origin = {8, 0}, extent = {{-80, -10}, {-60, 10}})));
  Modelica.Blocks.Sources.RealExpression angle(y=90) annotation (Placement(transformation(origin = {10, 0}, extent = {{-110, -10}, {-90, 10}})));
  AuxiliaryComponents.CraneCrab craneCrab1 annotation (Placement(transformation(origin = {-6, 0}, extent = {{18, -10}, {-2, 10}})));
  Modelica.Blocks.Math.InverseBlockConstraints inverseBlockConstraints
    annotation (Placement(transformation(origin = {-6, 0}, extent = {{-12, -10}, {28, 10}})));
  Modelica.Blocks.Math.UnitConversions.From_deg from_deg
    annotation (Placement(transformation(origin = {6, 0}, extent = {{-50, -10}, {-30, 10}})));
  AuxiliaryComponents.CraneCrab craneCrab(revolute(w(fixed=true)))
    annotation (Placement(transformation(origin = {-20, 0}, extent = {{60, -10}, {80, 10}})));
equation
  connect(inverseBlockConstraints.u2, craneCrab1.phi) annotation (Line(points={{-14, 0},{-9,0}}, color={0,0,127}));
  connect(craneCrab1.f, inverseBlockConstraints.y2) annotation (Line(points={{14,0},{19, 0}}, color={0,0,127}));
  connect(angle.y, secondOrder.u) annotation (Line(points={{-79,0},{-74,0}}, color={0,0,127}));
  connect(secondOrder.y, from_deg.u) annotation (Line(points={{-51,0},{-46,0}}, color={0,0,127}));
  connect(from_deg.y, inverseBlockConstraints.u1) annotation (Line(points={{-23,0},{-20, 0}},
                                                                                          color={0,0,127}));
  connect(inverseBlockConstraints.y1, craneCrab.f) annotation (Line(points={{23, 0},{38,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(__Dymola_NumberOfIntervals=5000, __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>
Example from Dirk's lecture. It does not make sense tho.
</p>
</html>"));
end Step7_InvertedCraneCrab2;
