Lambda Tango device
====================

This is the reference documentation of the Lambda Tango device.

you can also find some useful information about the camera models/prerequisite/installation/configuration/compilation in the :ref:`Lambda camera plugin <camera-lambda>` section.

Properties
----------

This camera device has no property.

=============== =============== =============== ==============================================================
Property name	Mandatory	Default value	Description
=============== =============== =============== ==============================================================
config_path     Yes              None           path the manufacturer configuration file of the detector
                                                should be something like: /opt/xsp/config
=============== =============== =============== ==============================================================

Attributes
----------
======================= ======= ======================= ============================================================
Attribute name		RW	Type			Description
======================= ======= ======================= ============================================================
distorsion_correction   ro      DevBoolean              Return **True**  if the distorsion correction is active
temperature             ro      DevDouble               The detector temperature in C
humidity                ro      DevDouble               The detector humitity in %
energy_threshold	rw	DevDouble		The energy threshold  in KeV
high_voltage		rw	DevDouble		The high voltage, relevant only for CdTe model
======================= ======= ======================= ============================================================

Distorsion_correction, temperature and humidity are only relevant with detector equiped with the latest harwdare 
and firmware, since mid of 2020.

Commands
--------

=======================	=============== =======================	======================================
Command name		Arg. in		Arg. out		Description
=======================	=============== =======================	======================================
Init			DevVoid 	DevVoid			Do not use
State			DevVoid		DevLong			Return the device state
Status			DevVoid		DevString		Return the device state as a string
getAttrStringValueList	DevString:	DevVarStringArray:	Return the authorized string value list for
			Attribute name	String value list	a given attribute name
=======================	=============== =======================	======================================


