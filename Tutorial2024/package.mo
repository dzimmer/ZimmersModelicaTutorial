within ;
package Tutorial2024 "Step by Step Modelica Tutorial"
  extends Modelica.Icons.Package;
  annotation (Icon(graphics={Text(
          extent={{-90,88},{90,-88}},
          textColor={28,108,200},
          textString="1,2,3")}), uses(
      PlanarMechanics(version="1.6.0"),
      Modelica(version="4.0.0"),
      ThermofluidStream(version="1.1.0")));
end Tutorial2024;
