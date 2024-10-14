within Tutorial2024.StepByStep;
model Step2_FixedBody "fixated body"
    extends Modelica.Icons.Example;

  inner PlanarMechanics.PlanarWorld planarWorld
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  PlanarMechanics.Parts.Body body(m=1, I=0.001)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  PlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,0})));
  PlanarMechanics.Parts.FixedTranslation rod(r={1,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation
  connect(rod.frame_b, body.frame_a) annotation (Line(
      points={{10,0},{20,0}},
      color={95,95,95},
      thickness=0.5));
  connect(fixed.frame, rod.frame_a) annotation (Line(
      points={{-20,0},{-10,0}},
      color={95,95,95},
      thickness=0.5));
  annotation (experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=1500,
      __Dymola_Algorithm="Dassl"));
end Step2_FixedBody;
