within Tutorial2015.BaseComponents;
package SimplePlanarMechanics "A planar mechanical library for didactical purposes"
extends Modelica.Icons.Package;
import SI = Modelica.SIunits;
import MB = Modelica.Mechanics.MultiBody;






  annotation (
  preferredView="info", Documentation(revisions="<html>
<p>Main author: Dr. Dirk Zimmer, Deutsches Zentrum f&uuml;r Luft- und Raumfahrt (DLR), Institute of System Dynamics and Control, 2015</p>
<p>Terms of usage: Usage of this library and its components is granted for non-commercial and educational purposes to everyone</p>
</html>", info="<html>
<p>Library <b>PlanarMechanics</b> is a <b>free</b> Modelica package providing 2-dimensional mechanical components to model mechanical systems, such as robots, mechanisms, vehicles, where MultiBody library is unnecessarily complex.</p>
<p>This variant is a heavily reduced and simplified variant of the free planar Mechanics library <a href=\"https://github.com/DLR-SR/PlanarMechanics\"> on  GitHub</a> </p>
</html>"),
    Icon(coordinateSystem(extent={{-100,-100},{100,100}}),
     graphics={
    Ellipse(
      extent={{-46,10},{-26,-8}},
      fillPattern=FillPattern.Solid,
      pattern=LinePattern.None),
    Ellipse(
      extent={{-74,-42},{-54,-60}},
      fillPattern=FillPattern.Solid,
      pattern=LinePattern.None),
    Ellipse(
      extent={{18,-34},{38,-52}},
      fillPattern=FillPattern.Solid,
      pattern=LinePattern.None),
    Line(
      points={{-64,-50},{-36,0},{24,-42}})}),
  Diagram(coordinateSystem(extent={{-120,-100},{80,100}})));
end SimplePlanarMechanics;
