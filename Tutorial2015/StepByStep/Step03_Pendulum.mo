within Tutorial2015.StepByStep;
model Step03_Pendulum "A simple pendulum"
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
  BaseComponents.SimplePlanarMechanics.Parts.FixedTranslation rod2(animate=
        true, r={1,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={10,10})));
  BaseComponents.SimplePlanarMechanics.Joints.Revolute revolute2(
    animate=true,
    w(fixed=true),
    useFlange=false,
    phi(fixed=true, start=0)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={-30,10})));
equation
  connect(revolute2.frame_a, fixed.frame_a) annotation (Line(
      points={{-40,10},{-60,10}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute2.frame_b, rod2.frame_a) annotation (Line(
      points={{-20,10},{0,10}},
      color={95,95,95},
      thickness=0.5));
  connect(rod2.frame_b, pendulum2.frame_a) annotation (Line(
      points={{20,10},{40,10}},
      color={95,95,95},
      thickness=0.5));
  annotation (experiment(StopTime=5),
    Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
end Step03_Pendulum;
