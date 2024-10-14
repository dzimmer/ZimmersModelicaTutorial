within Tutorial2024.StepByStep;
model Step1_FreeBody2 "free falling body"
    extends Modelica.Icons.Example;

  inner PlanarMechanics.PlanarWorld planarWorld
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  PlanarMechanics.Parts.Body body(m=1, I=0.001, r(start = {0, 0}), v(start = {5, 5}))
    annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
  annotation (experiment(
      StopTime=1,
      __Dymola_NumberOfIntervals=1500,
      __Dymola_Algorithm="Dassl"));
end Step1_FreeBody2;
