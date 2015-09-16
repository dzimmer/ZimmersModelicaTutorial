within Tutorial2015.BaseComponents.Friction;
model IdealDryFriction "Ideal dry (Coulomb) friction"
  extends Modelica.Mechanics.Translational.Interfaces.PartialCompliant;
  extends Modelica.Mechanics.Translational.Interfaces.PartialFriction(final v_small=1e-3);
  parameter Modelica.SIunits.Force S = 10 "Sticking friction force";
  parameter Modelica.SIunits.Force R = 8 "Sliding friction force";

equation
  free = false;
  f0 = R;
  f0_max = S;

  // velocity and acceleration of flanges
  v_relfric = der(s_rel);
  a_relfric = der(v_relfric);

  // Friction force
  f = if locked then sa*unitForce else (
     if startForward then R else if startBackward then -R else if pre(mode) == Forward then R else -R);

  annotation (                               Icon(graphics={
        Rectangle(
          extent={{-42,40},{58,12}},
          pattern=LinePattern.None,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-62,-12},{38,-40}},
          pattern=LinePattern.None,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-92,0},{-62,0},{-62,-40},{38,-40},{38,-12}},
          pattern=LinePattern.None),
        Line(
          points={{88,0},{58,0},{58,40},{-42,40},{-42,12}},
          pattern=LinePattern.None),
        Text(
          extent={{100,100},{-100,60}},
          textString="%name",
          lineColor={0,0,255})}), Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>"));
end IdealDryFriction;
