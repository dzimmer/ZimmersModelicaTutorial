within Tutorial2015.StepByStep;
model Step01_FreeBody "A free falling body"
  extends Modelica.Icons.Example;

  inner BaseComponents.SimplePlanarMechanics.PlanarWorld planarWorld(
    animateWorld=false,
    animateGravity=false)
    annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
  BaseComponents.SimplePlanarMechanics.Parts.Body freebody(
    m=1,
    animate=true,
    I=0.001,
    r(each fixed=true),
    v(each fixed=true),
    phi(fixed=true),
    w(fixed=true))
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  annotation (experiment(StopTime=1),
    Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
end Step01_FreeBody;
