within Tutorial2024.StepByStep;
model Step4_DoublePendulum "Double Pendulum"
    extends Modelica.Icons.Example;

  inner PlanarMechanics.PlanarWorld planarWorld
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  PlanarMechanics.Parts.Body body(m=1, I=0.001)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  PlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-60,0})));
  PlanarMechanics.Parts.FixedTranslation rod(r={1,0})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  PlanarMechanics.Joints.Revolute revolute(phi(start=1.3962634015955, fixed=
          true))
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  PlanarMechanics.Parts.Body body1(m=0.5, I=0.001)
    annotation (Placement(transformation(extent={{20,-40},{40,-20}})));
  PlanarMechanics.Parts.FixedTranslation rod1(r={0.5,0})
    annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));
  PlanarMechanics.Joints.Revolute revolute1
    annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
equation
  connect(rod.frame_b, body.frame_a) annotation (Line(
      points={{10,0},{20,0}},
      color={95,95,95},
      thickness=0.5));
  connect(fixed.frame, revolute.frame_a) annotation (Line(
      points={{-50,0},{-40,0}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_b, rod.frame_a) annotation (Line(
      points={{-20,0},{-10,0}},
      color={95,95,95},
      thickness=0.5));
  connect(rod1.frame_b, body1.frame_a) annotation (Line(
      points={{10,-30},{20,-30}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute1.frame_b, rod1.frame_a) annotation (Line(
      points={{-20,-30},{-10,-30}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute1.frame_a, body.frame_a) annotation (Line(
      points={{-40,-30},{-48,-30},{-48,-16},{16,-16},{16,0},{20,0}},
      color={95,95,95},
      thickness=0.5));
  annotation (experiment(
      StopTime=25,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end Step4_DoublePendulum;
