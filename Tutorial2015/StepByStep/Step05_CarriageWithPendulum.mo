within Tutorial2015.StepByStep;
model Step05_CarriageWithPendulum "A crane crab (carriage with pendulum)"
  extends Modelica.Icons.Example;

  inner BaseComponents.SimplePlanarMechanics.PlanarWorld planarWorld(
    animateWorld=false,
    animateGravity=false)
    annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
  BaseComponents.SimplePlanarMechanics.Parts.Body carriage(
    m=1,
    I=0,
    animate=true)
    annotation (Placement(transformation(extent={{40,0},{60,20}})));
  BaseComponents.SimplePlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-70,10})));
  BaseComponents.SimplePlanarMechanics.Parts.Body pendulum(
    I=0,
    m=1,
    animate=true) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={80,-30})));
  BaseComponents.SimplePlanarMechanics.Parts.FixedTranslation rod(r={0,0.5},
      animate=true) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={50,-30})));
  BaseComponents.SimplePlanarMechanics.Joints.Revolute revolute(
    animate=true,
    w(fixed=true),
    useFlange=false,
    phi(fixed=true, start=-0.087266462599716)) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,-10})));
  BaseComponents.SimplePlanarMechanics.Joints.Prismatic prismatic(
    r={1,0},
    animate=true,
    s(fixed=true),
    v(fixed=true),
    useFlange=false)
    annotation (Placement(transformation(extent={{-20,0},{0,20}})));
equation
  connect(rod.frame_b, pendulum.frame_a) annotation (Line(
      points={{60,-30},{70,-30}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_b, rod.frame_a) annotation (Line(
      points={{20,-20},{20,-30},{40,-30}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_a, carriage.frame_a) annotation (Line(
      points={{20,0},{20,10},{40,10}},
      color={95,95,95},
      thickness=0.5));
  connect(prismatic.frame_a, fixed.frame_a) annotation (Line(
      points={{-20,10},{-60,10}},
      color={95,95,95},
      thickness=0.5));
  connect(prismatic.frame_b, carriage.frame_a) annotation (Line(
      points={{0,10},{40,10}},
      color={95,95,95},
      thickness=0.5));
  annotation (experiment(StopTime=5),
    Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
end Step05_CarriageWithPendulum;
