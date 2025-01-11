# Tetwild MSH Converter

This application converts an MSH file produced by [Tetwild](https://github.com/daniel-zint/TetWild) into a VTU file.
If the MSH has an `in_out` attribute, it will convert it to a `winding_number` attribute.

#### Parameters

As this application is extremely simple, it does not require the standard JSON input. Instead, it just requires an input file name and optionally an output file name.

```
-i  Input MSH file
-j  Output VTU file (optional, by default the same as the input)
```

#### Examples of usage

```
./tetwild_msh_converter -i sphere.msh
```

This will create a `sphere_tets.vtu`.

```
./tetwild_msh_converter -i sphere.msh -o sphere_converted.vtu
```

This will create a `sphere_converted_tets.vtu`.

#### JSON parameter file

While a JSON parameter file is not necessary, it is still possible to be used with that, too. For details, look at `tetwild_msh_converter_spec.json`.
