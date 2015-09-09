within Tutorial2015.BaseComponents.SimplePlanarMechanics.Interfaces;
connector Frame_b
  extends Frame;
  annotation (Icon(graphics={
        Rectangle(
          extent={{-40,100},{40,-100}},
          lineColor={95,95,95},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Line(
          points={{-18,0},{22,0}},
          color={95,95,95}),
        Line(
          points={{0,20},{0,-20}},
          color={95,95,95})}), Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",
   info="<html>
<p>Frame_b is a connector, which lies at the origin of the coordinate system attached to it. Cut-force and cut_torque act at the origin of the coordinate system. Normally, this connector is fixed to a mechanical component. The same as <a href=\"modelica://PlanarMechanics.Interfaces.Frame_a\">Frame_a</a>.</p>
</html>"));
end Frame_b;
