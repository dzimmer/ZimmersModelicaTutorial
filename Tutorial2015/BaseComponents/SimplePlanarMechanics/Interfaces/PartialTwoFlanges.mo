within Tutorial2015.BaseComponents.SimplePlanarMechanics.Interfaces;
partial model PartialTwoFlanges "Partial model with 2 flanges"

  Frame_a frame_a
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
        iconTransformation(extent={{-120,-20},{-80,20}})));
  Frame_b frame_b
    annotation (Placement(transformation(extent={{90,-10},{110,10}}),
        iconTransformation(extent={{80,-20},{120,20}})));
  annotation (Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",
   info="<html>
<p>This is a partial model with 2 planar flanges. it can be inherited to build up models with 2 planar flanges.</p>
</html>"));
end PartialTwoFlanges;
