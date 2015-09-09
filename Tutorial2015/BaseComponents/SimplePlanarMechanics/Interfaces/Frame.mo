within Tutorial2015.BaseComponents.SimplePlanarMechanics.Interfaces;
connector Frame "General Connector for planar mechanical components"
  SI.Position x "x-position";
  SI.Position y "y-position";
  SI.Angle phi "angle (counter-clockwise)";
  flow SI.Force fx "force in x-direction";
  flow SI.Force fy "force in y-direction";
  flow SI.Torque t "torque (clockwise)";
  annotation (Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>",
   info="<html>
<p>Frame is a connector, which lies at the origin of the coordinate system attached to it. Cut-force and cut_torque act at the origin of the coordinate system. Normally, this connector is fixed to a mechanical component. But this model is never used directly in a system. It is only for usage of inheritance.</p>
</html>"));
end Frame;
