within Tutorial2024.StepByStep;
model Step2_FixedBody2 "fixated body"
    extends Modelica.Icons.Example;

  inner PlanarMechanics.PlanarWorld planarWorld
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  PlanarMechanics.Parts.Body body(m=1, I=0.001, r(start = {0, 0}))
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  PlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,0})));
  PlanarMechanics.Parts.SpringDamper springDamper(c_y = 10, d_y = 2*sqrt(10)/2)  annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));
equation
  connect(fixed.frame, springDamper.frame_a) annotation (
    Line(points = {{-20, 0}, {-10, 0}}, color = {95, 95, 95}));
  connect(springDamper.frame_b, body.frame_a) annotation (
    Line(points = {{10, 0}, {20, 0}}, color = {95, 95, 95}));
  annotation (experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=1500,
      __Dymola_Algorithm="Dassl"));
end Step2_FixedBody2;
