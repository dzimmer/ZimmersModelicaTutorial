within Tutorial2024.StepByStep;
model Step6_ActuatedCraneCrab "Actuated Crane Crab"
    extends Modelica.Icons.Example;

  inner PlanarMechanics.PlanarWorld planarWorld
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  PlanarMechanics.Parts.Body carriage(m=1, I=0.001)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  PlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-60,0})));
  PlanarMechanics.Parts.Body pendulum(m=0.5, I=0.001)
    annotation (Placement(transformation(extent={{40,-60},{60,-40}})));
  PlanarMechanics.Parts.FixedTranslation rod(r={0.5,0})
    annotation (Placement(transformation(extent={{10,-60},{30,-40}})));
  PlanarMechanics.Joints.Revolute revolute1
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-30})));
  PlanarMechanics.Joints.Prismatic prismatic(useFlange=true, r={1,0})
    annotation (Placement(transformation(extent={{-34,10},{-14,-10}})));
  Modelica.Mechanics.Translational.Sources.ConstantForce constantForce(
      f_constant=2)
    annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
equation
  connect(rod.frame_b, pendulum.frame_a) annotation (Line(
      points={{30,-50},{40,-50}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute1.frame_b, rod.frame_a) annotation (Line(
      points={{0,-40},{0,-50},{10,-50}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute1.frame_a, carriage.frame_a) annotation (Line(
      points={{0,-20},{0,0},{20,0}},
      color={95,95,95},
      thickness=0.5));
  connect(fixed.frame, prismatic.frame_a) annotation (Line(
      points={{-50,0},{-34,0}},
      color={95,95,95},
      thickness=0.5));
  connect(prismatic.frame_b, carriage.frame_a) annotation (Line(
      points={{-14,0},{20,0}},
      color={95,95,95},
      thickness=0.5));
  connect(constantForce.flange, prismatic.flange_a)
    annotation (Line(points={{-40,30},{-24,30},{-24,10}}, color={0,127,0}));
  annotation (experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=1500,
      __Dymola_Algorithm="Dassl"));
end Step6_ActuatedCraneCrab;
