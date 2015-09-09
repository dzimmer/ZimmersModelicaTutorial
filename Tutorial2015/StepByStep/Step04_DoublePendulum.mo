within Tutorial2015.StepByStep;
model Step04_DoublePendulum "A double pendulum"
  extends Modelica.Icons.Example;

  inner BaseComponents.SimplePlanarMechanics.PlanarWorld planarWorld(
    animateWorld=false,
    animateGravity=false)
    annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
  BaseComponents.SimplePlanarMechanics.Parts.Body pendulum2(
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
    phi(fixed=true, start=0)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,-10})));
  BaseComponents.SimplePlanarMechanics.Parts.FixedTranslation rod2(animate=
        true, r={1,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={-6,10})));
  BaseComponents.SimplePlanarMechanics.Joints.Revolute revolute2(
    animate=true,
    w(fixed=true),
    useFlange=false,
    phi(fixed=true, start=0)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={-38,10})));
equation
  connect(rod.frame_b, pendulum.frame_a) annotation (Line(
      points={{60,-30},{70,-30}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_b, rod.frame_a) annotation (Line(
      points={{20,-20},{20,-30},{40,-30}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_a, pendulum2.frame_a) annotation (Line(
      points={{20,0},{20,10},{40,10}},
      color={95,95,95},
      thickness=0.5));
  connect(rod2.frame_b, revolute.frame_a) annotation (Line(
      points={{4,10},{20,10},{20,0}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute2.frame_a, fixed.frame_a) annotation (Line(
      points={{-48,10},{-60,10}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute2.frame_b, rod2.frame_a) annotation (Line(
      points={{-28,10},{-16,10}},
      color={95,95,95},
      thickness=0.5));
  annotation (experiment(StopTime=15,
        Interval=0.01),
    Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
end Step04_DoublePendulum;
