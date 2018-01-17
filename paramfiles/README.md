# Parameter Files
---

Parameter files contain parameter values that the agent can load in at runtime.  Files should be formatted with a set of parameters as key value pairs from strings to floats.  The parameter name should be separated from its value with a tab (not a space) and parameters should be separated from each other with a single newline.  Parameter files support C++ style comments of `//` and `/* */` as well as `#`.

Parameter files are specified and loaded with the ```--paramsfile <parameter_file>``` command line argument, and multiple parameter files can be loaded one after the other with newly loaded parameter values replacing the values of previously loaded parameters with the same name (key).  Parameters are loaded into an `std::map` called `namedParams`.

All agents should first load the [defaultParams.txt](defaultParams.txt) parameter file, and then the appropriate *defaultParams_t&lt;type&gt;.txt* parameter file depending on an agent's body type, when starting and before loading any additional parameter files.
