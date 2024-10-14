within Tutorial2024.AuxiliaryComponents;
model ThermalCollector "collect all loads from a dc machine"
  Modelica.Electrical.Machines.Interfaces.DCMachines.ThermalPortDCPM                                                                          thermalPort
    annotation (Placement(transformation(extent={{-10,90},{10,110}}),
        iconTransformation(extent={{-10,90},{10,110}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a annotation (
      Placement(transformation(extent={{-10,-110},{10,-90}}),
        iconTransformation(extent={{-10,-110},{10,-90}})));
equation

  connect(port_a, thermalPort.heatPortArmature) annotation (Line(points={{0,-100},
          {0,2},{0,100},{0,100}},  color={191,0,0}));
  connect(port_a, thermalPort.heatPortCore) annotation (Line(points={{0,-100},
          {0,2},{0,100},{0,100}},
                               color={191,0,0}));
  connect(port_a, thermalPort.heatPortStrayLoad) annotation (Line(points={{0,-100},
          {0,0},{0,100},{0,100}}, color={191,0,0}));
  connect(port_a, thermalPort.heatPortFriction)
    annotation (Line(points={{0,-100},{0,-2},{0,100},{0,100}},
                                                             color={191,0,0}));
  connect(port_a, thermalPort.heatPortBrush)
    annotation (Line(points={{0,-100},{0,100}}, color={191,0,0}));
  connect(port_a, thermalPort.heatPortPermanentMagnet) annotation (Line(
        points={{0,-100},{0,-2},{0,98},{-1,98}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{0,-90},{0,-40}}, color={238,46,47}),
        Line(points={{0,-40},{-60,40}}, color={238,46,47}),
        Line(points={{0,-40},{60,40}}, color={238,46,47}),
        Line(points={{0,-40},{20,40}}, color={238,46,47}),
        Line(points={{0,-40},{-20,40}}, color={238,46,47}),
        Line(points={{-60,40},{-60,90}}, color={238,46,47}),
        Line(points={{-20,40},{-20,90}}, color={238,46,47}),
        Line(points={{20,40},{20,90}}, color={238,46,47}),
        Line(points={{60,40},{60,90}}, color={238,46,47})}),     Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ThermalCollector;
