within Tutorial2024.StepByStep;
model Step7_InvertedCraneCrab "Inverted Crane Crab"
    extends Modelica.Icons.Example;

  inner PlanarMechanics.PlanarWorld planarWorld
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  PlanarMechanics.Parts.Body carriage(m=1, I=0.001)
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  PlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-80,0})));
  PlanarMechanics.Parts.Body pendulum(m=0.5, I=0.001)
    annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
  PlanarMechanics.Parts.FixedTranslation rod(r={0,0.5})
    annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));
  PlanarMechanics.Joints.Revolute revolute1(useFlange=true)
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-20,-30})));
  PlanarMechanics.Joints.Prismatic prismatic(useFlange=true, r={1,0})
    annotation (Placement(transformation(extent={{-54,10},{-34,-10}})));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor
    annotation (Placement(transformation(extent={{-4,-40},{16,-20}})));
  Modelica.Blocks.Math.InverseBlockConstraints inverseBlockConstraints
    annotation (Placement(transformation(extent={{42,-50},{92,-10}})));
  Modelica.Blocks.Sources.Ramp ramp(
    startTime=0,
    duration=0.5,
    height=0.5,
    offset=-0.5)  annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=180,
        origin={80,-30})));
  Modelica.Blocks.Continuous.FirstOrder firstOrder(initType=Modelica.Blocks.Types.Init.SteadyState,
      T=0.1) annotation (Placement(transformation(extent={{66,-36},{54,-24}})));
  Modelica.Mechanics.Translational.Sources.Force force(useSupport=false)
    annotation (Placement(transformation(extent={{-20,30},{-40,50}})));
equation
  connect(rod.frame_b, pendulum.frame_a) annotation (Line(
      points={{10,-50},{20,-50}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute1.frame_b, rod.frame_a) annotation (Line(
      points={{-20,-40},{-20,-50},{-10,-50}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute1.frame_a, carriage.frame_a) annotation (Line(
      points={{-20,-20},{-20,0},{0,0}},
      color={95,95,95},
      thickness=0.5));
  connect(fixed.frame, prismatic.frame_a) annotation (Line(
      points={{-70,0},{-54,0}},
      color={95,95,95},
      thickness=0.5));
  connect(prismatic.frame_b, carriage.frame_a) annotation (Line(
      points={{-34,0},{0,0}},
      color={95,95,95},
      thickness=0.5));
  connect(inverseBlockConstraints.u1,angleSensor. phi) annotation (Line(
      points={{39.5,-30},{17,-30}},
      color={0,0,127}));
  connect(inverseBlockConstraints.y1,force. f) annotation (Line(
      points={{93.25,-30},{96,-30},{96,40},{-18,40}},
      color={0,0,127}));
  connect(ramp.y,firstOrder. u) annotation (Line(
      points={{73.4,-30},{67.2,-30}},
      color={0,0,127}));
  connect(firstOrder.y,inverseBlockConstraints. u2) annotation (Line(
      points={{53.4,-30},{47,-30}},
      color={0,0,127}));
  connect(force.flange, prismatic.flange_a)
    annotation (Line(points={{-40,40},{-44,40},{-44,10}}, color={0,127,0}));
  connect(revolute1.flange_a, angleSensor.flange)
    annotation (Line(points={{-10,-30},{-4,-30}}, color={0,0,0}));
  annotation (experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=1500,
      __Dymola_Algorithm="Dassl"));
end Step7_InvertedCraneCrab;
