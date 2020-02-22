{
    # Lift characterization
    # To run, type "pip install --upgrade frc-characterization"
    # Then, type "frc-characterization elevator new"
    # Then select this file
    # Warning: This project type is for BRUSHLESS motors ONLY!
    # Ports for the motors
    "motorPorts": [21],
    "motorPorts": [28],
    # NOTE: Inversions of the slaves (i.e. any motor *after* the first on
    # each side of the drive) are *with respect to their master*.  This is
    # different from the other poject types!
    # Inversions for the motors
    "motorsInverted": [True],
    # The total gear reduction between the motor and the pulley, expressed as
    # a fraction [motor turns]/[pulley turns]
    "gearing": 81/1,
    # Pulley diameter (in units of your choice - will dictate units of analysis)
    "pulleyDiameter": 1.87,
    # Offset of your encoder zero from horizontal
    "offset": 0,
}


