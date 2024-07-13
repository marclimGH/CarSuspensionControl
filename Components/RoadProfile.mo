package ModelicaServices "ModelicaServices (OpenModelica implementation) - Models and functions used in the Modelica Standard Library requiring a tool specific implementation"
  extends Modelica.Icons.Package;

  package Animation "Models and functions for 3-dim. animation"
    extends Modelica.Icons.Package;

    model Shape "Different visual shapes with variable size; all data have to be set as modifiers (see info layer)"
      extends Modelica.Utilities.Internal.PartialModelicaServices.Animation.PartialShape;
    end Shape;

    model Surface "Animation of a moveable, parameterized surface; the surface characteristic is provided by a function"
      extends Modelica.Utilities.Internal.PartialModelicaServices.Animation.PartialSurface;
    end Surface;
  end Animation;

  package Machine "Machine dependent constants"
    extends Modelica.Icons.Package;
    final constant Real eps = 1e-15 "Biggest number such that 1.0 + eps = 1.0";
    final constant Real small = 1e-60 "Smallest number such that small and -small are representable on the machine";
    final constant Real inf = 1e60 "Biggest Real number such that inf and -inf are representable on the machine";
    final constant Integer Integer_inf = OpenModelica.Internal.Architecture.integerMax() "Biggest Integer number such that Integer_inf and -Integer_inf are representable on the machine";
  end Machine;
  annotation(version = "4.0.0", versionDate = "2020-06-04", dateModified = "2020-06-04 11:00:00Z");
end ModelicaServices;

package Modelica "Modelica Standard Library - Version 4.0.0"
  extends Modelica.Icons.Package;

  package Blocks "Library of basic input/output control blocks (continuous, discrete, logical, table blocks)"
    extends Modelica.Icons.Package;
    import Modelica.Units.SI;

    package Continuous "Library of continuous control blocks with internal states"
      import Modelica.Blocks.Interfaces;
      extends Modelica.Icons.Package;

      block Filter "Continuous low pass, high pass, band pass or band stop IIR-filter of type CriticalDamping, Bessel, Butterworth or ChebyshevI"
        import Modelica.Blocks.Continuous.Internal;
        extends Modelica.Blocks.Interfaces.SISO;
        parameter Modelica.Blocks.Types.AnalogFilter analogFilter = Modelica.Blocks.Types.AnalogFilter.CriticalDamping "Analog filter characteristics (CriticalDamping/Bessel/Butterworth/ChebyshevI)";
        parameter Modelica.Blocks.Types.FilterType filterType = Modelica.Blocks.Types.FilterType.LowPass "Type of filter (LowPass/HighPass/BandPass/BandStop)";
        parameter Integer order(min = 1) = 2 "Order of filter";
        parameter SI.Frequency f_cut "Cut-off frequency";
        parameter Real gain = 1.0 "Gain (= amplitude of frequency response at zero frequency)";
        parameter Real A_ripple(unit = "dB") = 0.5 "Pass band ripple for Chebyshev filter (otherwise not used); > 0 required";
        parameter SI.Frequency f_min = 0 "Band of band pass/stop filter is f_min (A=-3db*gain) .. f_cut (A=-3db*gain)";
        parameter Boolean normalized = true "= true, if amplitude at f_cut = -3db, otherwise unmodified filter";
        parameter Modelica.Blocks.Types.Init init = Modelica.Blocks.Types.Init.SteadyState "Type of initialization (no init/steady state/initial state/initial output)" annotation(Evaluate = true);
        final parameter Integer nx = if filterType == Modelica.Blocks.Types.FilterType.LowPass or filterType == Modelica.Blocks.Types.FilterType.HighPass then order else 2 * order;
        parameter Real[nx] x_start = zeros(nx) "Initial or guess values of states";
        parameter Real y_start = 0 "Initial value of output";
        parameter Real u_nominal = 1.0 "Nominal value of input (used for scaling the states)";
        Modelica.Blocks.Interfaces.RealOutput[nx] x "Filter states";
      protected
        parameter Integer ncr = if analogFilter == Modelica.Blocks.Types.AnalogFilter.CriticalDamping then order else mod(order, 2);
        parameter Integer nc0 = if analogFilter == Modelica.Blocks.Types.AnalogFilter.CriticalDamping then 0 else integer(order / 2);
        parameter Integer na = if filterType == Modelica.Blocks.Types.FilterType.BandPass or filterType == Modelica.Blocks.Types.FilterType.BandStop then order else if analogFilter == Modelica.Blocks.Types.AnalogFilter.CriticalDamping then 0 else integer(order / 2);
        parameter Integer nr = if filterType == Modelica.Blocks.Types.FilterType.BandPass or filterType == Modelica.Blocks.Types.FilterType.BandStop then 0 else if analogFilter == Modelica.Blocks.Types.AnalogFilter.CriticalDamping then order else mod(order, 2);
        parameter Real[ncr] cr(each fixed = false);
        parameter Real[nc0] c0(each fixed = false);
        parameter Real[nc0] c1(each fixed = false);
        parameter Real[nr] r(each fixed = false);
        parameter Real[na] a(each fixed = false);
        parameter Real[na] b(each fixed = false);
        parameter Real[na] ku(each fixed = false);
        parameter Real[if filterType == Modelica.Blocks.Types.FilterType.LowPass then 0 else na] k1(each fixed = false);
        parameter Real[if filterType == Modelica.Blocks.Types.FilterType.LowPass then 0 else na] k2(each fixed = false);
        Real[na + nr + 1] uu;
      initial equation
        if analogFilter == Modelica.Blocks.Types.AnalogFilter.CriticalDamping then
          cr = Internal.Filter.base.CriticalDamping(order, normalized);
        elseif analogFilter == Modelica.Blocks.Types.AnalogFilter.Bessel then
          (cr, c0, c1) = Internal.Filter.base.Bessel(order, normalized);
        elseif analogFilter == Modelica.Blocks.Types.AnalogFilter.Butterworth then
          (cr, c0, c1) = Internal.Filter.base.Butterworth(order, normalized);
        elseif analogFilter == Modelica.Blocks.Types.AnalogFilter.ChebyshevI then
          (cr, c0, c1) = Internal.Filter.base.ChebyshevI(order, A_ripple, normalized);
        end if;
        if filterType == Modelica.Blocks.Types.FilterType.LowPass then
          (r, a, b, ku) = Internal.Filter.roots.lowPass(cr, c0, c1, f_cut);
        elseif filterType == Modelica.Blocks.Types.FilterType.HighPass then
          (r, a, b, ku, k1, k2) = Internal.Filter.roots.highPass(cr, c0, c1, f_cut);
        elseif filterType == Modelica.Blocks.Types.FilterType.BandPass then
          (a, b, ku, k1, k2) = Internal.Filter.roots.bandPass(cr, c0, c1, f_min, f_cut);
        elseif filterType == Modelica.Blocks.Types.FilterType.BandStop then
          (a, b, ku, k1, k2) = Internal.Filter.roots.bandStop(cr, c0, c1, f_min, f_cut);
        end if;
        if init == Modelica.Blocks.Types.Init.InitialState then
          x = x_start;
        elseif init == Modelica.Blocks.Types.Init.SteadyState then
          der(x) = zeros(nx);
        elseif init == Modelica.Blocks.Types.Init.InitialOutput then
          y = y_start;
          if nx > 1 then
            der(x[1:nx - 1]) = zeros(nx - 1);
          end if;
        end if;
      equation
        assert(u_nominal > 0, "u_nominal > 0 required");
        assert(filterType == Modelica.Blocks.Types.FilterType.LowPass or filterType == Modelica.Blocks.Types.FilterType.HighPass or f_min > 0, "f_min > 0 required for band pass and band stop filter");
        assert(A_ripple > 0, "A_ripple > 0 required");
        assert(f_cut > 0, "f_cut > 0 required");
        uu[1] = u / u_nominal;
        for i in 1:nr loop
          der(x[i]) = r[i] * (x[i] - uu[i]);
        end for;
        for i in 1:na loop
          der(x[nr + 2 * i - 1]) = a[i] * x[nr + 2 * i - 1] - b[i] * x[nr + 2 * i] + ku[i] * uu[nr + i];
          der(x[nr + 2 * i]) = b[i] * x[nr + 2 * i - 1] + a[i] * x[nr + 2 * i];
        end for;
        if filterType == Modelica.Blocks.Types.FilterType.LowPass then
          for i in 1:nr loop
            uu[i + 1] = x[i];
          end for;
          for i in 1:na loop
            uu[nr + i + 1] = x[nr + 2 * i];
          end for;
        elseif filterType == Modelica.Blocks.Types.FilterType.HighPass then
          for i in 1:nr loop
            uu[i + 1] = (-x[i]) + uu[i];
          end for;
          for i in 1:na loop
            uu[nr + i + 1] = k1[i] * x[nr + 2 * i - 1] + k2[i] * x[nr + 2 * i] + uu[nr + i];
          end for;
        elseif filterType == Modelica.Blocks.Types.FilterType.BandPass then
          for i in 1:na loop
            uu[nr + i + 1] = k1[i] * x[nr + 2 * i - 1] + k2[i] * x[nr + 2 * i];
          end for;
        elseif filterType == Modelica.Blocks.Types.FilterType.BandStop then
          for i in 1:na loop
            uu[nr + i + 1] = k1[i] * x[nr + 2 * i - 1] + k2[i] * x[nr + 2 * i] + uu[nr + i];
          end for;
        else
          assert(false, "filterType (= " + String(filterType) + ") is unknown");
          uu = zeros(na + nr + 1);
        end if;
        y = gain * u_nominal * uu[nr + na + 1];
      end Filter;

      package Internal "Internal utility functions and blocks that should not be directly utilized by the user"
        extends Modelica.Icons.InternalPackage;

        package Filter "Internal utility functions for filters that should not be directly used"
          extends Modelica.Icons.InternalPackage;

          package base "Prototype low pass filters with cut-off frequency of 1 rad/s (other filters are derived by transformation from these base filters)"
            extends Modelica.Icons.InternalPackage;

            function CriticalDamping "Return base filter coefficients of CriticalDamping filter (= low pass filter with w_cut = 1 rad/s)"
              extends Modelica.Icons.Function;
              input Integer order(min = 1) "Order of filter";
              input Boolean normalized = true "= true, if amplitude at f_cut = -3db, otherwise unmodified filter";
              output Real[order] cr "Coefficients of real poles";
            protected
              Real alpha = 1.0 "Frequency correction factor";
              Real alpha2 "= alpha*alpha";
              Real[order] den1 "[p] coefficients of denominator first order polynomials (a*p + 1)";
              Real[0, 2] den2 "[p^2, p] coefficients of denominator second order polynomials (b*p^2 + a*p + 1)";
              Real[0] c0 "Coefficients of s^0 term if conjugate complex pole";
              Real[0] c1 "Coefficients of s^1 term if conjugate complex pole";
            algorithm
              if normalized then
                alpha := sqrt(10 ^ (3 / 10 / order) - 1);
              else
                alpha := 1.0;
              end if;
              for i in 1:order loop
                den1[i] := alpha;
              end for;
              (cr, c0, c1) := Modelica.Blocks.Continuous.Internal.Filter.Utilities.toHighestPowerOne(den1, den2);
            end CriticalDamping;

            function Bessel "Return base filter coefficients of Bessel filter (= low pass filter with w_cut = 1 rad/s)"
              extends Modelica.Icons.Function;
              input Integer order(min = 1) "Order of filter";
              input Boolean normalized = true "= true, if amplitude at f_cut = -3db, otherwise unmodified filter";
              output Real[mod(order, 2)] cr "Coefficient of real pole";
              output Real[integer(order / 2)] c0 "Coefficients of s^0 term if conjugate complex pole";
              output Real[integer(order / 2)] c1 "Coefficients of s^1 term if conjugate complex pole";
            protected
              Real alpha = 1.0 "Frequency correction factor";
              Real alpha2 "= alpha*alpha";
              Real[size(cr, 1)] den1 "[p] coefficients of denominator first order polynomials (a*p + 1)";
              Real[size(c0, 1), 2] den2 "[p^2, p] coefficients of denominator second order polynomials (b*p^2 + a*p + 1)";
            algorithm
              (den1, den2, alpha) := Modelica.Blocks.Continuous.Internal.Filter.Utilities.BesselBaseCoefficients(order);
              if not normalized then
                alpha2 := alpha * alpha;
                for i in 1:size(c0, 1) loop
                  den2[i, 1] := den2[i, 1] * alpha2;
                  den2[i, 2] := den2[i, 2] * alpha;
                end for;
                if size(cr, 1) == 1 then
                  den1[1] := den1[1] * alpha;
                else
                end if;
              else
              end if;
              (cr, c0, c1) := Modelica.Blocks.Continuous.Internal.Filter.Utilities.toHighestPowerOne(den1, den2);
            end Bessel;

            function Butterworth "Return base filter coefficients of Butterworth filter (= low pass filter with w_cut = 1 rad/s)"
              import Modelica.Constants.pi;
              extends Modelica.Icons.Function;
              input Integer order(min = 1) "Order of filter";
              input Boolean normalized = true "= true, if amplitude at f_cut = -3db, otherwise unmodified filter";
              output Real[mod(order, 2)] cr "Coefficient of real pole";
              output Real[integer(order / 2)] c0 "Coefficients of s^0 term if conjugate complex pole";
              output Real[integer(order / 2)] c1 "Coefficients of s^1 term if conjugate complex pole";
            protected
              Real alpha = 1.0 "Frequency correction factor";
              Real alpha2 "= alpha*alpha";
              Real[size(cr, 1)] den1 "[p] coefficients of denominator first order polynomials (a*p + 1)";
              Real[size(c0, 1), 2] den2 "[p^2, p] coefficients of denominator second order polynomials (b*p^2 + a*p + 1)";
            algorithm
              for i in 1:size(c0, 1) loop
                den2[i, 1] := 1.0;
                den2[i, 2] := -2 * Modelica.Math.cos(pi * (0.5 + (i - 0.5) / order));
              end for;
              if size(cr, 1) == 1 then
                den1[1] := 1.0;
              else
              end if;
              (cr, c0, c1) := Modelica.Blocks.Continuous.Internal.Filter.Utilities.toHighestPowerOne(den1, den2);
            end Butterworth;

            function ChebyshevI "Return base filter coefficients of Chebyshev I filter (= low pass filter with w_cut = 1 rad/s)"
              import Modelica.Math.asinh;
              import Modelica.Constants.pi;
              extends Modelica.Icons.Function;
              input Integer order(min = 1) "Order of filter";
              input Real A_ripple = 0.5 "Pass band ripple in [dB]";
              input Boolean normalized = true "= true, if amplitude at f_cut = -3db, otherwise unmodified filter";
              output Real[mod(order, 2)] cr "Coefficient of real pole";
              output Real[integer(order / 2)] c0 "Coefficients of s^0 term if conjugate complex pole";
              output Real[integer(order / 2)] c1 "Coefficients of s^1 term if conjugate complex pole";
            protected
              Real epsilon;
              Real fac;
              Real alpha = 1.0 "Frequency correction factor";
              Real alpha2 "= alpha*alpha";
              Real[size(cr, 1)] den1 "[p] coefficients of denominator first order polynomials (a*p + 1)";
              Real[size(c0, 1), 2] den2 "[p^2, p] coefficients of denominator second order polynomials (b*p^2 + a*p + 1)";
            algorithm
              epsilon := sqrt(10 ^ (A_ripple / 10) - 1);
              fac := asinh(1 / epsilon) / order;
              den1 := fill(1 / sinh(fac), size(den1, 1));
              if size(cr, 1) == 0 then
                for i in 1:size(c0, 1) loop
                  den2[i, 1] := 1 / (cosh(fac) ^ 2 - cos((2 * i - 1) * pi / (2 * order)) ^ 2);
                  den2[i, 2] := 2 * den2[i, 1] * sinh(fac) * cos((2 * i - 1) * pi / (2 * order));
                end for;
              else
                for i in 1:size(c0, 1) loop
                  den2[i, 1] := 1 / (cosh(fac) ^ 2 - cos(i * pi / order) ^ 2);
                  den2[i, 2] := 2 * den2[i, 1] * sinh(fac) * cos(i * pi / order);
                end for;
              end if;
              if normalized then
                alpha := Modelica.Blocks.Continuous.Internal.Filter.Utilities.normalizationFactor(den1, den2);
                alpha2 := alpha * alpha;
                for i in 1:size(c0, 1) loop
                  den2[i, 1] := den2[i, 1] * alpha2;
                  den2[i, 2] := den2[i, 2] * alpha;
                end for;
                den1 := den1 * alpha;
              else
              end if;
              (cr, c0, c1) := Modelica.Blocks.Continuous.Internal.Filter.Utilities.toHighestPowerOne(den1, den2);
            end ChebyshevI;
          end base;

          package coefficients "Filter coefficients"
            extends Modelica.Icons.InternalPackage;

            function lowPass "Return low pass filter coefficients at given cut-off frequency"
              import Modelica.Constants.pi;
              extends Modelica.Icons.Function;
              input Real[:] cr_in "Coefficients of real poles";
              input Real[:] c0_in "Coefficients of s^0 term if conjugate complex pole";
              input Real[size(c0_in, 1)] c1_in "Coefficients of s^1 term if conjugate complex pole";
              input SI.Frequency f_cut "Cut-off frequency";
              output Real[size(cr_in, 1)] cr "Coefficient of real pole";
              output Real[size(c0_in, 1)] c0 "Coefficients of s^0 term if conjugate complex pole";
              output Real[size(c0_in, 1)] c1 "Coefficients of s^1 term if conjugate complex pole";
            protected
              SI.AngularVelocity w_cut = 2 * pi * f_cut "Cut-off angular frequency";
              Real w_cut2 = w_cut * w_cut;
            algorithm
              assert(f_cut > 0, "Cut-off frequency f_cut must be positive");
              cr := w_cut * cr_in;
              c1 := w_cut * c1_in;
              c0 := w_cut2 * c0_in;
            end lowPass;

            function highPass "Return high pass filter coefficients at given cut-off frequency"
              import Modelica.Constants.pi;
              extends Modelica.Icons.Function;
              input Real[:] cr_in "Coefficients of real poles";
              input Real[:] c0_in "Coefficients of s^0 term if conjugate complex pole";
              input Real[size(c0_in, 1)] c1_in "Coefficients of s^1 term if conjugate complex pole";
              input SI.Frequency f_cut "Cut-off frequency";
              output Real[size(cr_in, 1)] cr "Coefficient of real pole";
              output Real[size(c0_in, 1)] c0 "Coefficients of s^0 term if conjugate complex pole";
              output Real[size(c0_in, 1)] c1 "Coefficients of s^1 term if conjugate complex pole";
            protected
              SI.AngularVelocity w_cut = 2 * pi * f_cut "Cut-off angular frequency";
              Real w_cut2 = w_cut * w_cut;
            algorithm
              assert(f_cut > 0, "Cut-off frequency f_cut must be positive");
              for i in 1:size(cr_in, 1) loop
                cr[i] := w_cut / cr_in[i];
              end for;
              for i in 1:size(c0_in, 1) loop
                c0[i] := w_cut2 / c0_in[i];
                c1[i] := w_cut * c1_in[i] / c0_in[i];
              end for;
            end highPass;

            function bandPass "Return band pass filter coefficients at given cut-off frequency"
              import Modelica.Constants.pi;
              extends Modelica.Icons.Function;
              input Real[:] cr_in "Coefficients of real poles";
              input Real[:] c0_in "Coefficients of s^0 term if conjugate complex pole";
              input Real[size(c0_in, 1)] c1_in "Coefficients of s^1 term if conjugate complex pole";
              input SI.Frequency f_min "Band of band pass filter is f_min (A=-3db) .. f_max (A=-3db)";
              input SI.Frequency f_max "Upper band frequency";
              output Real[0] cr "Coefficient of real pole";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] c0 "Coefficients of s^0 term if conjugate complex pole";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] c1 "Coefficients of s^1 term if conjugate complex pole";
              output Real cn "Numerator coefficient of the PT2 terms";
            protected
              SI.Frequency f0 = sqrt(f_min * f_max);
              SI.AngularVelocity w_cut = 2 * pi * f0 "Cut-off angular frequency";
              Real w_band = (f_max - f_min) / f0;
              Real w_cut2 = w_cut * w_cut;
              Real c;
              Real alpha;
              Integer j;
            algorithm
              assert(f_min > 0 and f_min < f_max, "Band frequencies f_min and f_max are wrong");
              for i in 1:size(cr_in, 1) loop
                c1[i] := w_cut * cr_in[i] * w_band;
                c0[i] := w_cut2;
              end for;
              for i in 1:size(c1_in, 1) loop
                alpha := Modelica.Blocks.Continuous.Internal.Filter.Utilities.bandPassAlpha(c1_in[i], c0_in[i], w_band);
                c := c1_in[i] * w_band / (alpha + 1 / alpha);
                j := size(cr_in, 1) + 2 * i - 1;
                c1[j] := w_cut * c / alpha;
                c1[j + 1] := w_cut * c * alpha;
                c0[j] := w_cut2 / alpha ^ 2;
                c0[j + 1] := w_cut2 * alpha ^ 2;
              end for;
              cn := w_band * w_cut;
            end bandPass;

            function bandStop "Return band stop filter coefficients at given cut-off frequency"
              import Modelica.Constants.pi;
              extends Modelica.Icons.Function;
              input Real[:] cr_in "Coefficients of real poles";
              input Real[:] c0_in "Coefficients of s^0 term if conjugate complex pole";
              input Real[size(c0_in, 1)] c1_in "Coefficients of s^1 term if conjugate complex pole";
              input SI.Frequency f_min "Band of band stop filter is f_min (A=-3db) .. f_max (A=-3db)";
              input SI.Frequency f_max "Upper band frequency";
              output Real[0] cr "Coefficient of real pole";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] c0 "Coefficients of s^0 term if conjugate complex pole";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] c1 "Coefficients of s^1 term if conjugate complex pole";
            protected
              SI.Frequency f0 = sqrt(f_min * f_max);
              SI.AngularVelocity w_cut = 2 * pi * f0 "Cut-off angular frequency";
              Real w_band = (f_max - f_min) / f0;
              Real w_cut2 = w_cut * w_cut;
              Real c;
              Real ww;
              Real alpha;
              Integer j;
            algorithm
              assert(f_min > 0 and f_min < f_max, "Band frequencies f_min and f_max are wrong");
              for i in 1:size(cr_in, 1) loop
                c1[i] := w_cut * w_band / cr_in[i];
                c0[i] := w_cut2;
              end for;
              for i in 1:size(c1_in, 1) loop
                ww := w_band / c0_in[i];
                alpha := Modelica.Blocks.Continuous.Internal.Filter.Utilities.bandPassAlpha(c1_in[i], c0_in[i], ww);
                c := c1_in[i] * ww / (alpha + 1 / alpha);
                j := size(cr_in, 1) + 2 * i - 1;
                c1[j] := w_cut * c / alpha;
                c1[j + 1] := w_cut * c * alpha;
                c0[j] := w_cut2 / alpha ^ 2;
                c0[j + 1] := w_cut2 * alpha ^ 2;
              end for;
            end bandStop;
          end coefficients;

          package roots "Filter roots and gain as needed for block implementations"
            extends Modelica.Icons.InternalPackage;

            function lowPass "Return low pass filter roots as needed for block for given cut-off frequency"
              extends Modelica.Icons.Function;
              input Real[:] cr_in "Coefficients of real poles of base filter";
              input Real[:] c0_in "Coefficients of s^0 term of base filter if conjugate complex pole";
              input Real[size(c0_in, 1)] c1_in "Coefficients of s^1 term of base filter if conjugate complex pole";
              input SI.Frequency f_cut "Cut-off frequency";
              output Real[size(cr_in, 1)] r "Real eigenvalues";
              output Real[size(c0_in, 1)] a "Real parts of complex conjugate eigenvalues";
              output Real[size(c0_in, 1)] b "Imaginary parts of complex conjugate eigenvalues";
              output Real[size(c0_in, 1)] ku "Input gain";
            protected
              Real[size(c0_in, 1)] c0;
              Real[size(c0_in, 1)] c1;
              Real[size(cr_in, 1)] cr;
            algorithm
              (cr, c0, c1) := coefficients.lowPass(cr_in, c0_in, c1_in, f_cut);
              for i in 1:size(cr_in, 1) loop
                r[i] := -cr[i];
              end for;
              for i in 1:size(c0_in, 1) loop
                a[i] := -c1[i] / 2;
                b[i] := sqrt(c0[i] - a[i] * a[i]);
                ku[i] := c0[i] / b[i];
              end for;
            end lowPass;

            function highPass "Return high pass filter roots as needed for block for given cut-off frequency"
              extends Modelica.Icons.Function;
              input Real[:] cr_in "Coefficients of real poles of base filter";
              input Real[:] c0_in "Coefficients of s^0 term of base filter if conjugate complex pole";
              input Real[size(c0_in, 1)] c1_in "Coefficients of s^1 term of base filter if conjugate complex pole";
              input SI.Frequency f_cut "Cut-off frequency";
              output Real[size(cr_in, 1)] r "Real eigenvalues";
              output Real[size(c0_in, 1)] a "Real parts of complex conjugate eigenvalues";
              output Real[size(c0_in, 1)] b "Imaginary parts of complex conjugate eigenvalues";
              output Real[size(c0_in, 1)] ku "Gains of input terms";
              output Real[size(c0_in, 1)] k1 "Gains of y = k1*x1 + k2*x + u";
              output Real[size(c0_in, 1)] k2 "Gains of y = k1*x1 + k2*x + u";
            protected
              Real[size(c0_in, 1)] c0;
              Real[size(c0_in, 1)] c1;
              Real[size(cr_in, 1)] cr;
              Real ba2;
            algorithm
              (cr, c0, c1) := coefficients.highPass(cr_in, c0_in, c1_in, f_cut);
              for i in 1:size(cr_in, 1) loop
                r[i] := -cr[i];
              end for;
              for i in 1:size(c0_in, 1) loop
                a[i] := -c1[i] / 2;
                b[i] := sqrt(c0[i] - a[i] * a[i]);
                ku[i] := c0[i] / b[i];
                k1[i] := 2 * a[i] / ku[i];
                ba2 := (b[i] / a[i]) ^ 2;
                k2[i] := (1 - ba2) / (1 + ba2);
              end for;
            end highPass;

            function bandPass "Return band pass filter roots as needed for block for given cut-off frequency"
              extends Modelica.Icons.Function;
              input Real[:] cr_in "Coefficients of real poles of base filter";
              input Real[:] c0_in "Coefficients of s^0 term of base filter if conjugate complex pole";
              input Real[size(c0_in, 1)] c1_in "Coefficients of s^1 term of base filter if conjugate complex pole";
              input SI.Frequency f_min "Band of band pass filter is f_min (A=-3db) .. f_max (A=-3db)";
              input SI.Frequency f_max "Upper band frequency";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] a "Real parts of complex conjugate eigenvalues";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] b "Imaginary parts of complex conjugate eigenvalues";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] ku "Gains of input terms";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] k1 "Gains of y = k1*x1 + k2*x";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] k2 "Gains of y = k1*x1 + k2*x";
            protected
              Real[0] cr;
              Real[size(a, 1)] c0;
              Real[size(a, 1)] c1;
              Real cn;
              Real bb;
            algorithm
              (cr, c0, c1, cn) := coefficients.bandPass(cr_in, c0_in, c1_in, f_min, f_max);
              for i in 1:size(a, 1) loop
                a[i] := -c1[i] / 2;
                bb := c0[i] - a[i] * a[i];
                assert(bb >= 0, "\nNot possible to use band pass filter, since transformation results in\n" + "system that does not have conjugate complex poles.\n" + "Try to use another analog filter for the band pass.\n");
                b[i] := sqrt(bb);
                ku[i] := c0[i] / b[i];
                k1[i] := cn / ku[i];
                k2[i] := cn * a[i] / (b[i] * ku[i]);
              end for;
            end bandPass;

            function bandStop "Return band stop filter roots as needed for block for given cut-off frequency"
              extends Modelica.Icons.Function;
              input Real[:] cr_in "Coefficients of real poles of base filter";
              input Real[:] c0_in "Coefficients of s^0 term of base filter if conjugate complex pole";
              input Real[size(c0_in, 1)] c1_in "Coefficients of s^1 term of base filter if conjugate complex pole";
              input SI.Frequency f_min "Band of band stop filter is f_min (A=-3db) .. f_max (A=-3db)";
              input SI.Frequency f_max "Upper band frequency";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] a "Real parts of complex conjugate eigenvalues";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] b "Imaginary parts of complex conjugate eigenvalues";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] ku "Gains of input terms";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] k1 "Gains of y = k1*x1 + k2*x";
              output Real[size(cr_in, 1) + 2 * size(c0_in, 1)] k2 "Gains of y = k1*x1 + k2*x";
            protected
              Real[0] cr;
              Real[size(a, 1)] c0;
              Real[size(a, 1)] c1;
              Real cn;
              Real bb;
            algorithm
              (cr, c0, c1) := coefficients.bandStop(cr_in, c0_in, c1_in, f_min, f_max);
              for i in 1:size(a, 1) loop
                a[i] := -c1[i] / 2;
                bb := c0[i] - a[i] * a[i];
                assert(bb >= 0, "\nNot possible to use band stop filter, since transformation results in\n" + "system that does not have conjugate complex poles.\n" + "Try to use another analog filter for the band stop filter.\n");
                b[i] := sqrt(bb);
                ku[i] := c0[i] / b[i];
                k1[i] := 2 * a[i] / ku[i];
                k2[i] := (c0[i] + a[i] ^ 2 - b[i] ^ 2) / (b[i] * ku[i]);
              end for;
            end bandStop;
          end roots;

          package Utilities "Utility functions for filter computations"
            extends Modelica.Icons.InternalPackage;

            function BesselBaseCoefficients "Return coefficients of normalized low pass Bessel filter (= gain at cut-off frequency 1 rad/s is decreased 3dB)"
              extends Modelica.Icons.Function;
              import Modelica.Utilities.Streams;
              input Integer order "Order of filter in the range 1..41";
              output Real[mod(order, 2)] c1 "[p] coefficients of Bessel denominator polynomials (a*p + 1)";
              output Real[integer(order / 2), 2] c2 "[p^2, p] coefficients of Bessel denominator polynomials (b2*p^2 + b1*p + 1)";
              output Real alpha "Normalization factor";
            algorithm
              if order == 1 then
                alpha := 1.002377293007601;
                c1[1] := 0.9976283451109835;
              elseif order == 2 then
                alpha := 0.7356641785819585;
                c2[1, 1] := 0.6159132201783791;
                c2[1, 2] := 1.359315879600889;
              elseif order == 3 then
                alpha := 0.5704770156982642;
                c1[1] := 0.7548574865985343;
                c2[1, 1] := 0.4756958028827457;
                c2[1, 2] := 0.9980615136104388;
              elseif order == 4 then
                alpha := 0.4737978580281427;
                c2[1, 1] := 0.4873729247240677;
                c2[1, 2] := 1.337564170455762;
                c2[2, 1] := 0.3877724315741958;
                c2[2, 2] := 0.7730405590839861;
              elseif order == 5 then
                alpha := 0.4126226974763408;
                c1[1] := 0.6645723262620757;
                c2[1, 1] := 0.4115231900614016;
                c2[1, 2] := 1.138349926728708;
                c2[2, 1] := 0.3234938702877912;
                c2[2, 2] := 0.6205992985771313;
              elseif order == 6 then
                alpha := 0.3705098000736233;
                c2[1, 1] := 0.3874508649098960;
                c2[1, 2] := 1.219740879520741;
                c2[2, 1] := 0.3493298843155746;
                c2[2, 2] := 0.9670265529381365;
                c2[3, 1] := 0.2747419229514599;
                c2[3, 2] := 0.5122165075105700;
              elseif order == 7 then
                alpha := 0.3393452623586350;
                c1[1] := 0.5927147125821412;
                c2[1, 1] := 0.3383379423919174;
                c2[1, 2] := 1.092630816438030;
                c2[2, 1] := 0.3001025788696046;
                c2[2, 2] := 0.8289928256598656;
                c2[3, 1] := 0.2372867471539579;
                c2[3, 2] := 0.4325128641920154;
              elseif order == 8 then
                alpha := 0.3150267393795002;
                c2[1, 1] := 0.3151115975207653;
                c2[1, 2] := 1.109403015460190;
                c2[2, 1] := 0.2969344839572762;
                c2[2, 2] := 0.9737455812222699;
                c2[3, 1] := 0.2612545921889538;
                c2[3, 2] := 0.7190394712068573;
                c2[4, 1] := 0.2080523342974281;
                c2[4, 2] := 0.3721456473047434;
              elseif order == 9 then
                alpha := 0.2953310177184124;
                c1[1] := 0.5377196679501422;
                c2[1, 1] := 0.2824689124281034;
                c2[1, 2] := 1.022646191567475;
                c2[2, 1] := 0.2626824161383468;
                c2[2, 2] := 0.8695626454762596;
                c2[3, 1] := 0.2302781917677917;
                c2[3, 2] := 0.6309047553448520;
                c2[4, 1] := 0.1847991729757028;
                c2[4, 2] := 0.3251978031287202;
              elseif order == 10 then
                alpha := 0.2789426890619463;
                c2[1, 1] := 0.2640769908255582;
                c2[1, 2] := 1.019788132875305;
                c2[2, 1] := 0.2540802639216947;
                c2[2, 2] := 0.9377020417760623;
                c2[3, 1] := 0.2343577229427963;
                c2[3, 2] := 0.7802229808216112;
                c2[4, 1] := 0.2052193139338624;
                c2[4, 2] := 0.5594176813008133;
                c2[5, 1] := 0.1659546953748916;
                c2[5, 2] := 0.2878349616233292;
              elseif order == 11 then
                alpha := 0.2650227766037203;
                c1[1] := 0.4950265498954191;
                c2[1, 1] := 0.2411858478546218;
                c2[1, 2] := 0.9567800996387417;
                c2[2, 1] := 0.2296849355380925;
                c2[2, 2] := 0.8592523717113126;
                c2[3, 1] := 0.2107851705677406;
                c2[3, 2] := 0.7040216048898129;
                c2[4, 1] := 0.1846461385164021;
                c2[4, 2] := 0.5006729207276717;
                c2[5, 1] := 0.1504217970817433;
                c2[5, 2] := 0.2575070491320295;
              elseif order == 12 then
                alpha := 0.2530051198547209;
                c2[1, 1] := 0.2268294941204543;
                c2[1, 2] := 0.9473116570034053;
                c2[2, 1] := 0.2207657387793729;
                c2[2, 2] := 0.8933728946287606;
                c2[3, 1] := 0.2087600700376653;
                c2[3, 2] := 0.7886236252756229;
                c2[4, 1] := 0.1909959101492760;
                c2[4, 2] := 0.6389263649257017;
                c2[5, 1] := 0.1675208146048472;
                c2[5, 2] := 0.4517847275162215;
                c2[6, 1] := 0.1374257286372761;
                c2[6, 2] := 0.2324699157474680;
              elseif order == 13 then
                alpha := 0.2424910397561007;
                c1[1] := 0.4608848369928040;
                c2[1, 1] := 0.2099813050274780;
                c2[1, 2] := 0.8992478823790660;
                c2[2, 1] := 0.2027250423101359;
                c2[2, 2] := 0.8328117484224146;
                c2[3, 1] := 0.1907635894058731;
                c2[3, 2] := 0.7257379204691213;
                c2[4, 1] := 0.1742280397887686;
                c2[4, 2] := 0.5830640944868014;
                c2[5, 1] := 0.1530858190490478;
                c2[5, 2] := 0.4106192089751885;
                c2[6, 1] := 0.1264090712880446;
                c2[6, 2] := 0.2114980230156001;
              elseif order == 14 then
                alpha := 0.2331902368695848;
                c2[1, 1] := 0.1986162311411235;
                c2[1, 2] := 0.8876961808055535;
                c2[2, 1] := 0.1946683341271615;
                c2[2, 2] := 0.8500754229171967;
                c2[3, 1] := 0.1868331332895056;
                c2[3, 2] := 0.7764629313723603;
                c2[4, 1] := 0.1752118757862992;
                c2[4, 2] := 0.6699720402924552;
                c2[5, 1] := 0.1598906457908402;
                c2[5, 2] := 0.5348446712848934;
                c2[6, 1] := 0.1407810153019944;
                c2[6, 2] := 0.3755841316563539;
                c2[7, 1] := 0.1169627966707339;
                c2[7, 2] := 0.1937088226304455;
              elseif order == 15 then
                alpha := 0.2248854870552422;
                c1[1] := 0.4328492272335646;
                c2[1, 1] := 0.1857292591004588;
                c2[1, 2] := 0.8496337061962563;
                c2[2, 1] := 0.1808644178280136;
                c2[2, 2] := 0.8020517898136011;
                c2[3, 1] := 0.1728264404199081;
                c2[3, 2] := 0.7247449729331105;
                c2[4, 1] := 0.1616970125901954;
                c2[4, 2] := 0.6205369315943097;
                c2[5, 1] := 0.1475257264578426;
                c2[5, 2] := 0.4929612162355906;
                c2[6, 1] := 0.1301861023357119;
                c2[6, 2] := 0.3454770708040735;
                c2[7, 1] := 0.1087810777120188;
                c2[7, 2] := 0.1784526655428406;
              elseif order == 16 then
                alpha := 0.2174105053474761;
                c2[1, 1] := 0.1765637967473151;
                c2[1, 2] := 0.8377453068635511;
                c2[2, 1] := 0.1738525357503125;
                c2[2, 2] := 0.8102988957433199;
                c2[3, 1] := 0.1684627004613343;
                c2[3, 2] := 0.7563265923413258;
                c2[4, 1] := 0.1604519074815815;
                c2[4, 2] := 0.6776082294687619;
                c2[5, 1] := 0.1498828607802206;
                c2[5, 2] := 0.5766417034027680;
                c2[6, 1] := 0.1367764717792823;
                c2[6, 2] := 0.4563528264410489;
                c2[7, 1] := 0.1209810465419295;
                c2[7, 2] := 0.3193782657322374;
                c2[8, 1] := 0.1016312648007554;
                c2[8, 2] := 0.1652419227369036;
              elseif order == 17 then
                alpha := 0.2106355148193306;
                c1[1] := 0.4093223608497299;
                c2[1, 1] := 0.1664014345826274;
                c2[1, 2] := 0.8067173752345952;
                c2[2, 1] := 0.1629839591538256;
                c2[2, 2] := 0.7712924931447541;
                c2[3, 1] := 0.1573277802512491;
                c2[3, 2] := 0.7134213666303411;
                c2[4, 1] := 0.1494828185148637;
                c2[4, 2] := 0.6347841731714884;
                c2[5, 1] := 0.1394948812681826;
                c2[5, 2] := 0.5375594414619047;
                c2[6, 1] := 0.1273627583380806;
                c2[6, 2] := 0.4241608926375478;
                c2[7, 1] := 0.1129187258461290;
                c2[7, 2] := 0.2965752009703245;
                c2[8, 1] := 0.9533357359908857e-1;
                c2[8, 2] := 0.1537041700889585;
              elseif order == 18 then
                alpha := 0.2044575288651841;
                c2[1, 1] := 0.1588768571976356;
                c2[1, 2] := 0.7951914263212913;
                c2[2, 1] := 0.1569357024981854;
                c2[2, 2] := 0.7744529690772538;
                c2[3, 1] := 0.1530722206358810;
                c2[3, 2] := 0.7335304425992080;
                c2[4, 1] := 0.1473206710524167;
                c2[4, 2] := 0.6735038935387268;
                c2[5, 1] := 0.1397225420331520;
                c2[5, 2] := 0.5959151542621590;
                c2[6, 1] := 0.1303092459809849;
                c2[6, 2] := 0.5026483447894845;
                c2[7, 1] := 0.1190627367060072;
                c2[7, 2] := 0.3956893824587150;
                c2[8, 1] := 0.1058058030798994;
                c2[8, 2] := 0.2765091830730650;
                c2[9, 1] := 0.8974708108800873e-1;
                c2[9, 2] := 0.1435505288284833;
              elseif order == 19 then
                alpha := 0.1987936248083529;
                c1[1] := 0.3892259966869526;
                c2[1, 1] := 0.1506640012172225;
                c2[1, 2] := 0.7693121733774260;
                c2[2, 1] := 0.1481728062796673;
                c2[2, 2] := 0.7421133586741549;
                c2[3, 1] := 0.1440444668388838;
                c2[3, 2] := 0.6975075386214800;
                c2[4, 1] := 0.1383101628540374;
                c2[4, 2] := 0.6365464378910025;
                c2[5, 1] := 0.1310032283190998;
                c2[5, 2] := 0.5606211948462122;
                c2[6, 1] := 0.1221431166405330;
                c2[6, 2] := 0.4713530424221445;
                c2[7, 1] := 0.1116991161103884;
                c2[7, 2] := 0.3703717538617073;
                c2[8, 1] := 0.9948917351196349e-1;
                c2[8, 2] := 0.2587371155559744;
                c2[9, 1] := 0.8475989238107367e-1;
                c2[9, 2] := 0.1345537894555993;
              elseif order == 20 then
                alpha := 0.1935761760416219;
                c2[1, 1] := 0.1443871348337404;
                c2[1, 2] := 0.7584165598446141;
                c2[2, 1] := 0.1429501891353184;
                c2[2, 2] := 0.7423000962318863;
                c2[3, 1] := 0.1400877384920004;
                c2[3, 2] := 0.7104185332215555;
                c2[4, 1] := 0.1358210369491446;
                c2[4, 2] := 0.6634599783272630;
                c2[5, 1] := 0.1301773703034290;
                c2[5, 2] := 0.6024175491895959;
                c2[6, 1] := 0.1231826501439148;
                c2[6, 2] := 0.5285332736326852;
                c2[7, 1] := 0.1148465498575254;
                c2[7, 2] := 0.4431977385498628;
                c2[8, 1] := 0.1051289462376788;
                c2[8, 2] := 0.3477444062821162;
                c2[9, 1] := 0.9384622797485121e-1;
                c2[9, 2] := 0.2429038300327729;
                c2[10, 1] := 0.8028211612831444e-1;
                c2[10, 2] := 0.1265329974009533;
              elseif order == 21 then
                alpha := 0.1887494014766075;
                c1[1] := 0.3718070668941645;
                c2[1, 1] := 0.1376151928386445;
                c2[1, 2] := 0.7364290859445481;
                c2[2, 1] := 0.1357438914390695;
                c2[2, 2] := 0.7150167318935022;
                c2[3, 1] := 0.1326398453462415;
                c2[3, 2] := 0.6798001808470175;
                c2[4, 1] := 0.1283231214897678;
                c2[4, 2] := 0.6314663440439816;
                c2[5, 1] := 0.1228169159777534;
                c2[5, 2] := 0.5709353626166905;
                c2[6, 1] := 0.1161406100773184;
                c2[6, 2] := 0.4993087153571335;
                c2[7, 1] := 0.1082959649233524;
                c2[7, 2] := 0.4177766148584385;
                c2[8, 1] := 0.9923596957485723e-1;
                c2[8, 2] := 0.3274257287232124;
                c2[9, 1] := 0.8877776108724853e-1;
                c2[9, 2] := 0.2287218166767916;
                c2[10, 1] := 0.7624076527736326e-1;
                c2[10, 2] := 0.1193423971506988;
              elseif order == 22 then
                alpha := 0.1842668221199706;
                c2[1, 1] := 0.1323053462701543;
                c2[1, 2] := 0.7262446126765204;
                c2[2, 1] := 0.1312121721769772;
                c2[2, 2] := 0.7134286088450949;
                c2[3, 1] := 0.1290330911166814;
                c2[3, 2] := 0.6880287870435514;
                c2[4, 1] := 0.1257817990372067;
                c2[4, 2] := 0.6505015800059301;
                c2[5, 1] := 0.1214765261983008;
                c2[5, 2] := 0.6015107185211451;
                c2[6, 1] := 0.1161365140967959;
                c2[6, 2] := 0.5418983553698413;
                c2[7, 1] := 0.1097755171533100;
                c2[7, 2] := 0.4726370779831614;
                c2[8, 1] := 0.1023889478519956;
                c2[8, 2] := 0.3947439506537486;
                c2[9, 1] := 0.9392485861253800e-1;
                c2[9, 2] := 0.3090996703083202;
                c2[10, 1] := 0.8420273775456455e-1;
                c2[10, 2] := 0.2159561978556017;
                c2[11, 1] := 0.7257600023938262e-1;
                c2[11, 2] := 0.1128633732721116;
              elseif order == 23 then
                alpha := 0.1800893554453722;
                c1[1] := 0.3565232673929280;
                c2[1, 1] := 0.1266275171652706;
                c2[1, 2] := 0.7072778066734162;
                c2[2, 1] := 0.1251865227648538;
                c2[2, 2] := 0.6900676345785905;
                c2[3, 1] := 0.1227944815236645;
                c2[3, 2] := 0.6617011100576023;
                c2[4, 1] := 0.1194647013077667;
                c2[4, 2] := 0.6226432315773119;
                c2[5, 1] := 0.1152132989252356;
                c2[5, 2] := 0.5735222810625359;
                c2[6, 1] := 0.1100558598478487;
                c2[6, 2] := 0.5151027978024605;
                c2[7, 1] := 0.1040013558214886;
                c2[7, 2] := 0.4482410942032739;
                c2[8, 1] := 0.9704014176512626e-1;
                c2[8, 2] := 0.3738049984631116;
                c2[9, 1] := 0.8911683905758054e-1;
                c2[9, 2] := 0.2925028692588410;
                c2[10, 1] := 0.8005438265072295e-1;
                c2[10, 2] := 0.2044134600278901;
                c2[11, 1] := 0.6923832296800832e-1;
                c2[11, 2] := 0.1069984887283394;
              elseif order == 24 then
                alpha := 0.1761838665838427;
                c2[1, 1] := 0.1220804912720132;
                c2[1, 2] := 0.6978026874156063;
                c2[2, 1] := 0.1212296762358897;
                c2[2, 2] := 0.6874139794926736;
                c2[3, 1] := 0.1195328372961027;
                c2[3, 2] := 0.6667954259551859;
                c2[4, 1] := 0.1169990987333593;
                c2[4, 2] := 0.6362602049901176;
                c2[5, 1] := 0.1136409040480130;
                c2[5, 2] := 0.5962662188435553;
                c2[6, 1] := 0.1094722001757955;
                c2[6, 2] := 0.5474001634109253;
                c2[7, 1] := 0.1045052832229087;
                c2[7, 2] := 0.4903523180249535;
                c2[8, 1] := 0.9874509806025907e-1;
                c2[8, 2] := 0.4258751523524645;
                c2[9, 1] := 0.9217799943472177e-1;
                c2[9, 2] := 0.3547079765396403;
                c2[10, 1] := 0.8474633796250476e-1;
                c2[10, 2] := 0.2774145482392767;
                c2[11, 1] := 0.7627722381240495e-1;
                c2[11, 2] := 0.1939329108084139;
                c2[12, 1] := 0.6618645465422745e-1;
                c2[12, 2] := 0.1016670147947242;
              elseif order == 25 then
                alpha := 0.1725220521949266;
                c1[1] := 0.3429735385896000;
                c2[1, 1] := 0.1172525033170618;
                c2[1, 2] := 0.6812327932576614;
                c2[2, 1] := 0.1161194585333535;
                c2[2, 2] := 0.6671566071153211;
                c2[3, 1] := 0.1142375145794466;
                c2[3, 2] := 0.6439167855053158;
                c2[4, 1] := 0.1116157454252308;
                c2[4, 2] := 0.6118378416180135;
                c2[5, 1] := 0.1082654809459177;
                c2[5, 2] := 0.5713609763370088;
                c2[6, 1] := 0.1041985674230918;
                c2[6, 2] := 0.5230289949762722;
                c2[7, 1] := 0.9942439308123559e-1;
                c2[7, 2] := 0.4674627926041906;
                c2[8, 1] := 0.9394453593830893e-1;
                c2[8, 2] := 0.4053226688298811;
                c2[9, 1] := 0.8774221237222533e-1;
                c2[9, 2] := 0.3372372276379071;
                c2[10, 1] := 0.8075839512216483e-1;
                c2[10, 2] := 0.2636485508005428;
                c2[11, 1] := 0.7282483286646764e-1;
                c2[11, 2] := 0.1843801345273085;
                c2[12, 1] := 0.6338571166846652e-1;
                c2[12, 2] := 0.9680153764737715e-1;
              elseif order == 26 then
                alpha := 0.1690795702796737;
                c2[1, 1] := 0.1133168695796030;
                c2[1, 2] := 0.6724297955493932;
                c2[2, 1] := 0.1126417845769961;
                c2[2, 2] := 0.6638709519790540;
                c2[3, 1] := 0.1112948749545606;
                c2[3, 2] := 0.6468652038763624;
                c2[4, 1] := 0.1092823986944244;
                c2[4, 2] := 0.6216337070799265;
                c2[5, 1] := 0.1066130386697976;
                c2[5, 2] := 0.5885011413992190;
                c2[6, 1] := 0.1032969057045413;
                c2[6, 2] := 0.5478864278297548;
                c2[7, 1] := 0.9934388184210715e-1;
                c2[7, 2] := 0.5002885306054287;
                c2[8, 1] := 0.9476081523436283e-1;
                c2[8, 2] := 0.4462644847551711;
                c2[9, 1] := 0.8954648464575577e-1;
                c2[9, 2] := 0.3863930785049522;
                c2[10, 1] := 0.8368166847159917e-1;
                c2[10, 2] := 0.3212074592527143;
                c2[11, 1] := 0.7710664731701103e-1;
                c2[11, 2] := 0.2510470347119383;
                c2[12, 1] := 0.6965807988411425e-1;
                c2[12, 2] := 0.1756419294111342;
                c2[13, 1] := 0.6080674930548766e-1;
                c2[13, 2] := 0.9234535279274277e-1;
              elseif order == 27 then
                alpha := 0.1658353543067995;
                c1[1] := 0.3308543720638957;
                c2[1, 1] := 0.1091618578712746;
                c2[1, 2] := 0.6577977071169651;
                c2[2, 1] := 0.1082549561495043;
                c2[2, 2] := 0.6461121666520275;
                c2[3, 1] := 0.1067479247890451;
                c2[3, 2] := 0.6267937760991321;
                c2[4, 1] := 0.1046471079537577;
                c2[4, 2] := 0.6000750116745808;
                c2[5, 1] := 0.1019605976654259;
                c2[5, 2] := 0.5662734183049320;
                c2[6, 1] := 0.9869726954433709e-1;
                c2[6, 2] := 0.5257827234948534;
                c2[7, 1] := 0.9486520934132483e-1;
                c2[7, 2] := 0.4790595019077763;
                c2[8, 1] := 0.9046906518775348e-1;
                c2[8, 2] := 0.4266025862147336;
                c2[9, 1] := 0.8550529998276152e-1;
                c2[9, 2] := 0.3689188223512328;
                c2[10, 1] := 0.7995282239306020e-1;
                c2[10, 2] := 0.3064589322702932;
                c2[11, 1] := 0.7375174596252882e-1;
                c2[11, 2] := 0.2394754504667310;
                c2[12, 1] := 0.6674377263329041e-1;
                c2[12, 2] := 0.1676223546666024;
                c2[13, 1] := 0.5842458027529246e-1;
                c2[13, 2] := 0.8825044329219431e-1;
              elseif order == 28 then
                alpha := 0.1627710671942929;
                c2[1, 1] := 0.1057232656113488;
                c2[1, 2] := 0.6496161226860832;
                c2[2, 1] := 0.1051786825724864;
                c2[2, 2] := 0.6424661279909941;
                c2[3, 1] := 0.1040917964935006;
                c2[3, 2] := 0.6282470268918791;
                c2[4, 1] := 0.1024670101953951;
                c2[4, 2] := 0.6071189030701136;
                c2[5, 1] := 0.1003105109519892;
                c2[5, 2] := 0.5793175191747016;
                c2[6, 1] := 0.9762969425430802e-1;
                c2[6, 2] := 0.5451486608855443;
                c2[7, 1] := 0.9443223803058400e-1;
                c2[7, 2] := 0.5049796971628137;
                c2[8, 1] := 0.9072460982036488e-1;
                c2[8, 2] := 0.4592270546572523;
                c2[9, 1] := 0.8650956423253280e-1;
                c2[9, 2] := 0.4083368605952977;
                c2[10, 1] := 0.8178165740374893e-1;
                c2[10, 2] := 0.3527525188880655;
                c2[11, 1] := 0.7651838885868020e-1;
                c2[11, 2] := 0.2928534570013572;
                c2[12, 1] := 0.7066010532447490e-1;
                c2[12, 2] := 0.2288185204390681;
                c2[13, 1] := 0.6405358596145789e-1;
                c2[13, 2] := 0.1602396172588190;
                c2[14, 1] := 0.5621780070227172e-1;
                c2[14, 2] := 0.8447589564915071e-1;
              elseif order == 29 then
                alpha := 0.1598706626277596;
                c1[1] := 0.3199314513011623;
                c2[1, 1] := 0.1021101032532951;
                c2[1, 2] := 0.6365758882240111;
                c2[2, 1] := 0.1013729819392774;
                c2[2, 2] := 0.6267495975736321;
                c2[3, 1] := 0.1001476175660628;
                c2[3, 2] := 0.6104876178266819;
                c2[4, 1] := 0.9843854640428316e-1;
                c2[4, 2] := 0.5879603139195113;
                c2[5, 1] := 0.9625164534591696e-1;
                c2[5, 2] := 0.5594012291050210;
                c2[6, 1] := 0.9359356960417668e-1;
                c2[6, 2] := 0.5251016150410664;
                c2[7, 1] := 0.9047086748649986e-1;
                c2[7, 2] := 0.4854024475590397;
                c2[8, 1] := 0.8688856407189167e-1;
                c2[8, 2] := 0.4406826457109709;
                c2[9, 1] := 0.8284779224069856e-1;
                c2[9, 2] := 0.3913408089298914;
                c2[10, 1] := 0.7834154620997181e-1;
                c2[10, 2] := 0.3377643999400627;
                c2[11, 1] := 0.7334628941928766e-1;
                c2[11, 2] := 0.2802710651919946;
                c2[12, 1] := 0.6780290487362146e-1;
                c2[12, 2] := 0.2189770008083379;
                c2[13, 1] := 0.6156321231528423e-1;
                c2[13, 2] := 0.1534235999306070;
                c2[14, 1] := 0.5416797446761512e-1;
                c2[14, 2] := 0.8098664736760292e-1;
              elseif order == 30 then
                alpha := 0.1571200296252450;
                c2[1, 1] := 0.9908074847842124e-1;
                c2[1, 2] := 0.6289618807831557;
                c2[2, 1] := 0.9863509708328196e-1;
                c2[2, 2] := 0.6229164525571278;
                c2[3, 1] := 0.9774542692037148e-1;
                c2[3, 2] := 0.6108853364240036;
                c2[4, 1] := 0.9641490581986484e-1;
                c2[4, 2] := 0.5929869253412513;
                c2[5, 1] := 0.9464802912225441e-1;
                c2[5, 2] := 0.5693960175547550;
                c2[6, 1] := 0.9245027206218041e-1;
                c2[6, 2] := 0.5403402396359503;
                c2[7, 1] := 0.8982754584112941e-1;
                c2[7, 2] := 0.5060948065875106;
                c2[8, 1] := 0.8678535291732599e-1;
                c2[8, 2] := 0.4669749797983789;
                c2[9, 1] := 0.8332744242052199e-1;
                c2[9, 2] := 0.4233249626334694;
                c2[10, 1] := 0.7945356393775309e-1;
                c2[10, 2] := 0.3755006094498054;
                c2[11, 1] := 0.7515543969833788e-1;
                c2[11, 2] := 0.3238400339292700;
                c2[12, 1] := 0.7040879901685638e-1;
                c2[12, 2] := 0.2686072427439079;
                c2[13, 1] := 0.6515528854010540e-1;
                c2[13, 2] := 0.2098650589782619;
                c2[14, 1] := 0.5925168237177876e-1;
                c2[14, 2] := 0.1471138832654873;
                c2[15, 1] := 0.5225913954211672e-1;
                c2[15, 2] := 0.7775248839507864e-1;
              elseif order == 31 then
                alpha := 0.1545067022920929;
                c1[1] := 0.3100206996451866;
                c2[1, 1] := 0.9591020358831668e-1;
                c2[1, 2] := 0.6172474793293396;
                c2[2, 1] := 0.9530301275601203e-1;
                c2[2, 2] := 0.6088916323460413;
                c2[3, 1] := 0.9429332655402368e-1;
                c2[3, 2] := 0.5950511595503025;
                c2[4, 1] := 0.9288445429894548e-1;
                c2[4, 2] := 0.5758534119053522;
                c2[5, 1] := 0.9108073420087422e-1;
                c2[5, 2] := 0.5514734636081183;
                c2[6, 1] := 0.8888719137536870e-1;
                c2[6, 2] := 0.5221306199481831;
                c2[7, 1] := 0.8630901440239650e-1;
                c2[7, 2] := 0.4880834248148061;
                c2[8, 1] := 0.8335074993373294e-1;
                c2[8, 2] := 0.4496225358496770;
                c2[9, 1] := 0.8001502494376102e-1;
                c2[9, 2] := 0.4070602306679052;
                c2[10, 1] := 0.7630041338037624e-1;
                c2[10, 2] := 0.3607139804818122;
                c2[11, 1] := 0.7219760885744920e-1;
                c2[11, 2] := 0.3108783301229550;
                c2[12, 1] := 0.6768185077153345e-1;
                c2[12, 2] := 0.2577706252514497;
                c2[13, 1] := 0.6269571766328638e-1;
                c2[13, 2] := 0.2014081375889921;
                c2[14, 1] := 0.5710081766945065e-1;
                c2[14, 2] := 0.1412581515841926;
                c2[15, 1] := 0.5047740914807019e-1;
                c2[15, 2] := 0.7474725873250158e-1;
              elseif order == 32 then
                alpha := 0.1520196210848210;
                c2[1, 1] := 0.9322163554339406e-1;
                c2[1, 2] := 0.6101488690506050;
                c2[2, 1] := 0.9285233997694042e-1;
                c2[2, 2] := 0.6049832320721264;
                c2[3, 1] := 0.9211494244473163e-1;
                c2[3, 2] := 0.5946969295569034;
                c2[4, 1] := 0.9101176786042449e-1;
                c2[4, 2] := 0.5793791854364477;
                c2[5, 1] := 0.8954614071360517e-1;
                c2[5, 2] := 0.5591619969234026;
                c2[6, 1] := 0.8772216763680164e-1;
                c2[6, 2] := 0.5342177994699602;
                c2[7, 1] := 0.8554440426912734e-1;
                c2[7, 2] := 0.5047560942986598;
                c2[8, 1] := 0.8301735302045588e-1;
                c2[8, 2] := 0.4710187048140929;
                c2[9, 1] := 0.8014469519188161e-1;
                c2[9, 2] := 0.4332730387207936;
                c2[10, 1] := 0.7692807528893225e-1;
                c2[10, 2] := 0.3918021436411035;
                c2[11, 1] := 0.7336507157284898e-1;
                c2[11, 2] := 0.3468890521471250;
                c2[12, 1] := 0.6944555312763458e-1;
                c2[12, 2] := 0.2987898029050460;
                c2[13, 1] := 0.6514446669420571e-1;
                c2[13, 2] := 0.2476810747407199;
                c2[14, 1] := 0.6040544477732702e-1;
                c2[14, 2] := 0.1935412053397663;
                c2[15, 1] := 0.5509478650672775e-1;
                c2[15, 2] := 0.1358108994174911;
                c2[16, 1] := 0.4881064725720192e-1;
                c2[16, 2] := 0.7194819894416505e-1;
              elseif order == 33 then
                alpha := 0.1496489351138032;
                c1[1] := 0.3009752799176432;
                c2[1, 1] := 0.9041725460994505e-1;
                c2[1, 2] := 0.5995521047364046;
                c2[2, 1] := 0.8991117804113002e-1;
                c2[2, 2] := 0.5923764112099496;
                c2[3, 1] := 0.8906941547422532e-1;
                c2[3, 2] := 0.5804822013853129;
                c2[4, 1] := 0.8789442491445575e-1;
                c2[4, 2] := 0.5639663528946501;
                c2[5, 1] := 0.8638945831033775e-1;
                c2[5, 2] := 0.5429623519607796;
                c2[6, 1] := 0.8455834602616358e-1;
                c2[6, 2] := 0.5176379938389326;
                c2[7, 1] := 0.8240517431382334e-1;
                c2[7, 2] := 0.4881921474066189;
                c2[8, 1] := 0.7993380417355076e-1;
                c2[8, 2] := 0.4548502528082586;
                c2[9, 1] := 0.7714713890732801e-1;
                c2[9, 2] := 0.4178579388038483;
                c2[10, 1] := 0.7404596598181127e-1;
                c2[10, 2] := 0.3774715722484659;
                c2[11, 1] := 0.7062702339160462e-1;
                c2[11, 2] := 0.3339432938810453;
                c2[12, 1] := 0.6687952672391507e-1;
                c2[12, 2] := 0.2874950693388235;
                c2[13, 1] := 0.6277828912909767e-1;
                c2[13, 2] := 0.2382680702894708;
                c2[14, 1] := 0.5826808305383988e-1;
                c2[14, 2] := 0.1862073169968455;
                c2[15, 1] := 0.5321974125363517e-1;
                c2[15, 2] := 0.1307323751236313;
                c2[16, 1] := 0.4724820282032780e-1;
                c2[16, 2] := 0.6933542082177094e-1;
              elseif order == 34 then
                alpha := 0.1473858373968463;
                c2[1, 1] := 0.8801537152275983e-1;
                c2[1, 2] := 0.5929204288972172;
                c2[2, 1] := 0.8770594341007476e-1;
                c2[2, 2] := 0.5884653382247518;
                c2[3, 1] := 0.8708797598072095e-1;
                c2[3, 2] := 0.5795895850253119;
                c2[4, 1] := 0.8616320590689187e-1;
                c2[4, 2] := 0.5663615383647170;
                c2[5, 1] := 0.8493413175570858e-1;
                c2[5, 2] := 0.5488825092350877;
                c2[6, 1] := 0.8340387368687513e-1;
                c2[6, 2] := 0.5272851839324592;
                c2[7, 1] := 0.8157596213131521e-1;
                c2[7, 2] := 0.5017313864372913;
                c2[8, 1] := 0.7945402670834270e-1;
                c2[8, 2] := 0.4724089864574216;
                c2[9, 1] := 0.7704133559556429e-1;
                c2[9, 2] := 0.4395276256463053;
                c2[10, 1] := 0.7434009635219704e-1;
                c2[10, 2] := 0.4033126590648964;
                c2[11, 1] := 0.7135035113853376e-1;
                c2[11, 2] := 0.3639961488919042;
                c2[12, 1] := 0.6806813160738834e-1;
                c2[12, 2] := 0.3218025212900124;
                c2[13, 1] := 0.6448214312000864e-1;
                c2[13, 2] := 0.2769235521088158;
                c2[14, 1] := 0.6056719318430530e-1;
                c2[14, 2] := 0.2294693573271038;
                c2[15, 1] := 0.5626925196925040e-1;
                c2[15, 2] := 0.1793564218840015;
                c2[16, 1] := 0.5146352031547277e-1;
                c2[16, 2] := 0.1259877129326412;
                c2[17, 1] := 0.4578069074410591e-1;
                c2[17, 2] := 0.6689147319568768e-1;
              elseif order == 35 then
                alpha := 0.1452224267615486;
                c1[1] := 0.2926764667564367;
                c2[1, 1] := 0.8551731299267280e-1;
                c2[1, 2] := 0.5832758214629523;
                c2[2, 1] := 0.8509109732853060e-1;
                c2[2, 2] := 0.5770596582643844;
                c2[3, 1] := 0.8438201446671953e-1;
                c2[3, 2] := 0.5667497616665494;
                c2[4, 1] := 0.8339191981579831e-1;
                c2[4, 2] := 0.5524209816238369;
                c2[5, 1] := 0.8212328610083385e-1;
                c2[5, 2] := 0.5341766459916322;
                c2[6, 1] := 0.8057906332198853e-1;
                c2[6, 2] := 0.5121470053512750;
                c2[7, 1] := 0.7876247299954955e-1;
                c2[7, 2] := 0.4864870722254752;
                c2[8, 1] := 0.7667670879950268e-1;
                c2[8, 2] := 0.4573736721705665;
                c2[9, 1] := 0.7432449556218945e-1;
                c2[9, 2] := 0.4250013835198991;
                c2[10, 1] := 0.7170742126011575e-1;
                c2[10, 2] := 0.3895767735915445;
                c2[11, 1] := 0.6882488171701314e-1;
                c2[11, 2] := 0.3513097926737368;
                c2[12, 1] := 0.6567231746957568e-1;
                c2[12, 2] := 0.3103999917596611;
                c2[13, 1] := 0.6223804362223595e-1;
                c2[13, 2] := 0.2670123611280899;
                c2[14, 1] := 0.5849696460782910e-1;
                c2[14, 2] := 0.2212298104867592;
                c2[15, 1] := 0.5439628409499822e-1;
                c2[15, 2] := 0.1729443731341637;
                c2[16, 1] := 0.4981540179136920e-1;
                c2[16, 2] := 0.1215462157134930;
                c2[17, 1] := 0.4439981033536435e-1;
                c2[17, 2] := 0.6460098363520967e-1;
              elseif order == 36 then
                alpha := 0.1431515914458580;
                c2[1, 1] := 0.8335881847130301e-1;
                c2[1, 2] := 0.5770670512160201;
                c2[2, 1] := 0.8309698922852212e-1;
                c2[2, 2] := 0.5731929100172432;
                c2[3, 1] := 0.8257400347039723e-1;
                c2[3, 2] := 0.5654713811993058;
                c2[4, 1] := 0.8179117911600136e-1;
                c2[4, 2] := 0.5539556343603020;
                c2[5, 1] := 0.8075042173126963e-1;
                c2[5, 2] := 0.5387245649546684;
                c2[6, 1] := 0.7945413151258206e-1;
                c2[6, 2] := 0.5198817177723069;
                c2[7, 1] := 0.7790506514288866e-1;
                c2[7, 2] := 0.4975537629595409;
                c2[8, 1] := 0.7610613635339480e-1;
                c2[8, 2] := 0.4718884193866789;
                c2[9, 1] := 0.7406012816626425e-1;
                c2[9, 2] := 0.4430516443136726;
                c2[10, 1] := 0.7176927060205631e-1;
                c2[10, 2] := 0.4112237708115829;
                c2[11, 1] := 0.6923460172504251e-1;
                c2[11, 2] := 0.3765940116389730;
                c2[12, 1] := 0.6645495833489556e-1;
                c2[12, 2] := 0.3393522147815403;
                c2[13, 1] := 0.6342528888937094e-1;
                c2[13, 2] := 0.2996755899575573;
                c2[14, 1] := 0.6013361864949449e-1;
                c2[14, 2] := 0.2577053294053830;
                c2[15, 1] := 0.5655503081322404e-1;
                c2[15, 2] := 0.2135004731531631;
                c2[16, 1] := 0.5263798119559069e-1;
                c2[16, 2] := 0.1669320999865636;
                c2[17, 1] := 0.4826589873626196e-1;
                c2[17, 2] := 0.1173807590715484;
                c2[18, 1] := 0.4309819397289806e-1;
                c2[18, 2] := 0.6245036108880222e-1;
              elseif order == 37 then
                alpha := 0.1411669104782917;
                c1[1] := 0.2850271036215707;
                c2[1, 1] := 0.8111958235023328e-1;
                c2[1, 2] := 0.5682412610563970;
                c2[2, 1] := 0.8075727567979578e-1;
                c2[2, 2] := 0.5628142923227016;
                c2[3, 1] := 0.8015440554413301e-1;
                c2[3, 2] := 0.5538087696879930;
                c2[4, 1] := 0.7931239302677386e-1;
                c2[4, 2] := 0.5412833323304460;
                c2[5, 1] := 0.7823314328639347e-1;
                c2[5, 2] := 0.5253190555393968;
                c2[6, 1] := 0.7691895211595101e-1;
                c2[6, 2] := 0.5060183741977191;
                c2[7, 1] := 0.7537237072011853e-1;
                c2[7, 2] := 0.4835036020049034;
                c2[8, 1] := 0.7359601294804538e-1;
                c2[8, 2] := 0.4579149413954837;
                c2[9, 1] := 0.7159227884849299e-1;
                c2[9, 2] := 0.4294078049978829;
                c2[10, 1] := 0.6936295002846032e-1;
                c2[10, 2] := 0.3981491350382047;
                c2[11, 1] := 0.6690857785828917e-1;
                c2[11, 2] := 0.3643121502867948;
                c2[12, 1] := 0.6422751692085542e-1;
                c2[12, 2] := 0.3280684291406284;
                c2[13, 1] := 0.6131430866206096e-1;
                c2[13, 2] := 0.2895750997170303;
                c2[14, 1] := 0.5815677249570920e-1;
                c2[14, 2] := 0.2489521814805720;
                c2[15, 1] := 0.5473023527947980e-1;
                c2[15, 2] := 0.2062377435955363;
                c2[16, 1] := 0.5098441033167034e-1;
                c2[16, 2] := 0.1612849131645336;
                c2[17, 1] := 0.4680658811093562e-1;
                c2[17, 2] := 0.1134672937045305;
                c2[18, 1] := 0.4186928031694695e-1;
                c2[18, 2] := 0.6042754777339966e-1;
              elseif order == 38 then
                alpha := 0.1392625697140030;
                c2[1, 1] := 0.7916943373658329e-1;
                c2[1, 2] := 0.5624158631591745;
                c2[2, 1] := 0.7894592250257840e-1;
                c2[2, 2] := 0.5590219398777304;
                c2[3, 1] := 0.7849941672384930e-1;
                c2[3, 2] := 0.5522551628416841;
                c2[4, 1] := 0.7783093084875645e-1;
                c2[4, 2] := 0.5421574325808380;
                c2[5, 1] := 0.7694193770482690e-1;
                c2[5, 2] := 0.5287909941093643;
                c2[6, 1] := 0.7583430534712885e-1;
                c2[6, 2] := 0.5122376814029880;
                c2[7, 1] := 0.7451020436122948e-1;
                c2[7, 2] := 0.4925978555548549;
                c2[8, 1] := 0.7297197617673508e-1;
                c2[8, 2] := 0.4699889739625235;
                c2[9, 1] := 0.7122194706992953e-1;
                c2[9, 2] := 0.4445436860615774;
                c2[10, 1] := 0.6926216260386816e-1;
                c2[10, 2] := 0.4164072786327193;
                c2[11, 1] := 0.6709399961255503e-1;
                c2[11, 2] := 0.3857341621868851;
                c2[12, 1] := 0.6471757977022456e-1;
                c2[12, 2] := 0.3526828388476838;
                c2[13, 1] := 0.6213084287116965e-1;
                c2[13, 2] := 0.3174082831364342;
                c2[14, 1] := 0.5932799638550641e-1;
                c2[14, 2] := 0.2800495563550299;
                c2[15, 1] := 0.5629672408524944e-1;
                c2[15, 2] := 0.2407078154782509;
                c2[16, 1] := 0.5301264751544952e-1;
                c2[16, 2] := 0.1994026830553859;
                c2[17, 1] := 0.4942673259817896e-1;
                c2[17, 2] := 0.1559719194038917;
                c2[18, 1] := 0.4542996716979947e-1;
                c2[18, 2] := 0.1097844277878470;
                c2[19, 1] := 0.4070720755433961e-1;
                c2[19, 2] := 0.5852181110523043e-1;
              elseif order == 39 then
                alpha := 0.1374332900196804;
                c1[1] := 0.2779468246419593;
                c2[1, 1] := 0.7715084161825772e-1;
                c2[1, 2] := 0.5543001331300056;
                c2[2, 1] := 0.7684028301163326e-1;
                c2[2, 2] := 0.5495289890712267;
                c2[3, 1] := 0.7632343924866024e-1;
                c2[3, 2] := 0.5416083298429741;
                c2[4, 1] := 0.7560141319808483e-1;
                c2[4, 2] := 0.5305846713929198;
                c2[5, 1] := 0.7467569064745969e-1;
                c2[5, 2] := 0.5165224112570647;
                c2[6, 1] := 0.7354807648551346e-1;
                c2[6, 2] := 0.4995030679271456;
                c2[7, 1] := 0.7222060351121389e-1;
                c2[7, 2] := 0.4796242430956156;
                c2[8, 1] := 0.7069540462458585e-1;
                c2[8, 2] := 0.4569982440368368;
                c2[9, 1] := 0.6897453353492381e-1;
                c2[9, 2] := 0.4317502624832354;
                c2[10, 1] := 0.6705970959388781e-1;
                c2[10, 2] := 0.4040159353969854;
                c2[11, 1] := 0.6495194541066725e-1;
                c2[11, 2] := 0.3739379843169939;
                c2[12, 1] := 0.6265098412417610e-1;
                c2[12, 2] := 0.3416613843816217;
                c2[13, 1] := 0.6015440984955930e-1;
                c2[13, 2] := 0.3073260166338746;
                c2[14, 1] := 0.5745615876877304e-1;
                c2[14, 2] := 0.2710546723961181;
                c2[15, 1] := 0.5454383762391338e-1;
                c2[15, 2] := 0.2329316824061170;
                c2[16, 1] := 0.5139340231935751e-1;
                c2[16, 2] := 0.1929604256043231;
                c2[17, 1] := 0.4795705862458131e-1;
                c2[17, 2] := 0.1509655259246037;
                c2[18, 1] := 0.4412933231935506e-1;
                c2[18, 2] := 0.1063130748962878;
                c2[19, 1] := 0.3960672309405603e-1;
                c2[19, 2] := 0.5672356837211527e-1;
              elseif order == 40 then
                alpha := 0.1356742655825434;
                c2[1, 1] := 0.7538038374294594e-1;
                c2[1, 2] := 0.5488228264329617;
                c2[2, 1] := 0.7518806529402738e-1;
                c2[2, 2] := 0.5458297722483311;
                c2[3, 1] := 0.7480383050347119e-1;
                c2[3, 2] := 0.5398604576730540;
                c2[4, 1] := 0.7422847031965465e-1;
                c2[4, 2] := 0.5309482987446206;
                c2[5, 1] := 0.7346313704205006e-1;
                c2[5, 2] := 0.5191429845322307;
                c2[6, 1] := 0.7250930053201402e-1;
                c2[6, 2] := 0.5045099368431007;
                c2[7, 1] := 0.7136868456879621e-1;
                c2[7, 2] := 0.4871295553902607;
                c2[8, 1] := 0.7004317764946634e-1;
                c2[8, 2] := 0.4670962098860498;
                c2[9, 1] := 0.6853470921527828e-1;
                c2[9, 2] := 0.4445169164956202;
                c2[10, 1] := 0.6684507689945471e-1;
                c2[10, 2] := 0.4195095960479698;
                c2[11, 1] := 0.6497570123412630e-1;
                c2[11, 2] := 0.3922007419030645;
                c2[12, 1] := 0.6292726794917847e-1;
                c2[12, 2] := 0.3627221993494397;
                c2[13, 1] := 0.6069918741663154e-1;
                c2[13, 2] := 0.3312065181294388;
                c2[14, 1] := 0.5828873983769410e-1;
                c2[14, 2] := 0.2977798532686911;
                c2[15, 1] := 0.5568964389813015e-1;
                c2[15, 2] := 0.2625503293999835;
                c2[16, 1] := 0.5288947816690705e-1;
                c2[16, 2] := 0.2255872486520188;
                c2[17, 1] := 0.4986456327645859e-1;
                c2[17, 2] := 0.1868796731919594;
                c2[18, 1] := 0.4656832613054458e-1;
                c2[18, 2] := 0.1462410193532463;
                c2[19, 1] := 0.4289867647614935e-1;
                c2[19, 2] := 0.1030361558710747;
                c2[20, 1] := 0.3856310684054106e-1;
                c2[20, 2] := 0.5502423832293889e-1;
              elseif order == 41 then
                alpha := 0.1339811106984253;
                c1[1] := 0.2713685065531391;
                c2[1, 1] := 0.7355140275160984e-1;
                c2[1, 2] := 0.5413274778282860;
                c2[2, 1] := 0.7328319082267173e-1;
                c2[2, 2] := 0.5371064088294270;
                c2[3, 1] := 0.7283676160772547e-1;
                c2[3, 2] := 0.5300963437270770;
                c2[4, 1] := 0.7221298133014343e-1;
                c2[4, 2] := 0.5203345998371490;
                c2[5, 1] := 0.7141302173623395e-1;
                c2[5, 2] := 0.5078728971879841;
                c2[6, 1] := 0.7043831559982149e-1;
                c2[6, 2] := 0.4927768111819803;
                c2[7, 1] := 0.6929049381827268e-1;
                c2[7, 2] := 0.4751250308594139;
                c2[8, 1] := 0.6797129849758392e-1;
                c2[8, 2] := 0.4550083840638406;
                c2[9, 1] := 0.6648246325101609e-1;
                c2[9, 2] := 0.4325285673076087;
                c2[10, 1] := 0.6482554675958526e-1;
                c2[10, 2] := 0.4077964789091151;
                c2[11, 1] := 0.6300169683004558e-1;
                c2[11, 2] := 0.3809299858742483;
                c2[12, 1] := 0.6101130648543355e-1;
                c2[12, 2] := 0.3520508315700898;
                c2[13, 1] := 0.5885349417435808e-1;
                c2[13, 2] := 0.3212801560701271;
                c2[14, 1] := 0.5652528148656809e-1;
                c2[14, 2] := 0.2887316252774887;
                c2[15, 1] := 0.5402021575818373e-1;
                c2[15, 2] := 0.2545001287790888;
                c2[16, 1] := 0.5132588802608274e-1;
                c2[16, 2] := 0.2186415296842951;
                c2[17, 1] := 0.4841900639702602e-1;
                c2[17, 2] := 0.1811322622296060;
                c2[18, 1] := 0.4525419574485134e-1;
                c2[18, 2] := 0.1417762065404688;
                c2[19, 1] := 0.4173260173087802e-1;
                c2[19, 2] := 0.9993834530966510e-1;
                c2[20, 1] := 0.3757210572966463e-1;
                c2[20, 2] := 0.5341611499960143e-1;
              else
                Streams.error("Input argument order (= " + String(order) + ") of Bessel filter is not in the range 1..41");
              end if;
            end BesselBaseCoefficients;

            function toHighestPowerOne "Transform filter to form with highest power of s equal 1"
              extends Modelica.Icons.Function;
              input Real[:] den1 "[s] coefficients of polynomials (den1[i]*s + 1)";
              input Real[:, 2] den2 "[s^2, s] coefficients of polynomials (den2[i,1]*s^2 + den2[i,2]*s + 1)";
              output Real[size(den1, 1)] cr "[s^0] coefficients of polynomials cr[i]*(s+1/cr[i])";
              output Real[size(den2, 1)] c0 "[s^0] coefficients of polynomials (s^2 + (den2[i,2]/den2[i,1])*s + (1/den2[i,1]))";
              output Real[size(den2, 1)] c1 "[s^1] coefficients of polynomials (s^2 + (den2[i,2]/den2[i,1])*s + (1/den2[i,1]))";
            algorithm
              for i in 1:size(den1, 1) loop
                cr[i] := 1 / den1[i];
              end for;
              for i in 1:size(den2, 1) loop
                c1[i] := den2[i, 2] / den2[i, 1];
                c0[i] := 1 / den2[i, 1];
              end for;
            end toHighestPowerOne;

            function normalizationFactor "Compute correction factor of low pass filter such that amplitude at cut-off frequency is -3db (=10^(-3/20) = 0.70794...)"
              extends Modelica.Icons.Function;
              import Modelica.Utilities.Streams;
              input Real[:] c1 "[p] coefficients of denominator polynomials (c1[i}*p + 1)";
              input Real[:, 2] c2 "[p^2, p] coefficients of denominator polynomials (c2[i,1]*p^2 + c2[i,2]*p + 1)";
              output Real alpha "Correction factor (replace p by alpha*p)";
            protected
              Real alpha_min;
              Real alpha_max;

              function normalizationResidue "Residue of correction factor computation"
                extends Modelica.Icons.Function;
                input Real[:] c1 "[p] coefficients of denominator polynomials (c1[i]*p + 1)";
                input Real[:, 2] c2 "[p^2, p] coefficients of denominator polynomials (c2[i,1]*p^2 + c2[i,2]*p + 1)";
                input Real alpha;
                output Real residue;
              protected
                constant Real beta = 10 ^ (-3 / 20) "Amplitude of -3db required, i.e., -3db = 20*log(beta)";
                Real cc1;
                Real cc2;
                Real p;
                Real alpha2 = alpha * alpha;
                Real alpha4 = alpha2 * alpha2;
                Real A2 = 1.0;
              algorithm
                assert(size(c1, 1) <= 1, "Internal error 2 (should not occur)");
                if size(c1, 1) == 1 then
                  cc1 := c1[1] * c1[1];
                  p := 1 + cc1 * alpha2;
                  A2 := A2 * p;
                else
                end if;
                for i in 1:size(c2, 1) loop
                  cc1 := c2[i, 2] * c2[i, 2] - 2 * c2[i, 1];
                  cc2 := c2[i, 1] * c2[i, 1];
                  p := 1 + cc1 * alpha2 + cc2 * alpha4;
                  A2 := A2 * p;
                end for;
                residue := 1 / sqrt(A2) - beta;
              end normalizationResidue;

              function findInterval "Find interval for the root"
                extends Modelica.Icons.Function;
                input Real[:] c1 "[p] coefficients of denominator polynomials (a*p + 1)";
                input Real[:, 2] c2 "[p^2, p] coefficients of denominator polynomials (b*p^2 + a*p + 1)";
                output Real alpha_min;
                output Real alpha_max;
              protected
                Real alpha = 1.0;
                Real residue;
              algorithm
                alpha_min := 0;
                residue := normalizationResidue(c1, c2, alpha);
                if residue < 0 then
                  alpha_max := alpha;
                else
                  while residue >= 0 loop
                    alpha := 1.1 * alpha;
                    residue := normalizationResidue(c1, c2, alpha);
                  end while;
                  alpha_max := alpha;
                end if;
              end findInterval;

              function solveOneNonlinearEquation "Solve f(u) = 0; f(u_min) and f(u_max) must have different signs"
                extends Modelica.Icons.Function;
                import Modelica.Utilities.Streams.error;
                input Real[:] c1 "[p] coefficients of denominator polynomials (c1[i]*p + 1)";
                input Real[:, 2] c2 "[p^2, p] coefficients of denominator polynomials (c2[i,1]*p^2 + c2[i,2]*p + 1)";
                input Real u_min "Lower bound of search interval";
                input Real u_max "Upper bound of search interval";
                input Real tolerance = 100 * Modelica.Constants.eps "Relative tolerance of solution u";
                output Real u "Value of independent variable so that f(u) = 0";
              protected
                constant Real eps = Modelica.Constants.eps "Machine epsilon";
                Real a = u_min "Current best minimum interval value";
                Real b = u_max "Current best maximum interval value";
                Real c "Intermediate point a <= c <= b";
                Real d;
                Real e "b - a";
                Real m;
                Real s;
                Real p;
                Real q;
                Real r;
                Real tol;
                Real fa "= f(a)";
                Real fb "= f(b)";
                Real fc;
                Boolean found = false;
              algorithm
                fa := normalizationResidue(c1, c2, u_min);
                fb := normalizationResidue(c1, c2, u_max);
                fc := fb;
                if fa > 0.0 and fb > 0.0 or fa < 0.0 and fb < 0.0 then
                  error("The arguments u_min and u_max to solveOneNonlinearEquation(..)\n" + "do not bracket the root of the single non-linear equation:\n" + "  u_min  = " + String(u_min) + "\n" + "  u_max  = " + String(u_max) + "\n" + "  fa = f(u_min) = " + String(fa) + "\n" + "  fb = f(u_max) = " + String(fb) + "\n" + "fa and fb must have opposite sign which is not the case");
                else
                end if;
                c := a;
                fc := fa;
                e := b - a;
                d := e;
                while not found loop
                  if abs(fc) < abs(fb) then
                    a := b;
                    b := c;
                    c := a;
                    fa := fb;
                    fb := fc;
                    fc := fa;
                  else
                  end if;
                  tol := 2 * eps * abs(b) + tolerance;
                  m := (c - b) / 2;
                  if abs(m) <= tol or fb == 0.0 then
                    found := true;
                    u := b;
                  else
                    if abs(e) < tol or abs(fa) <= abs(fb) then
                      e := m;
                      d := e;
                    else
                      s := fb / fa;
                      if a == c then
                        p := 2 * m * s;
                        q := 1 - s;
                      else
                        q := fa / fc;
                        r := fb / fc;
                        p := s * (2 * m * q * (q - r) - (b - a) * (r - 1));
                        q := (q - 1) * (r - 1) * (s - 1);
                      end if;
                      if p > 0 then
                        q := -q;
                      else
                        p := -p;
                      end if;
                      s := e;
                      e := d;
                      if 2 * p < 3 * m * q - abs(tol * q) and p < abs(0.5 * s * q) then
                        d := p / q;
                      else
                        e := m;
                        d := e;
                      end if;
                    end if;
                    a := b;
                    fa := fb;
                    b := b + (if abs(d) > tol then d else if m > 0 then tol else -tol);
                    fb := normalizationResidue(c1, c2, b);
                    if fb > 0 and fc > 0 or fb < 0 and fc < 0 then
                      c := a;
                      fc := fa;
                      e := b - a;
                      d := e;
                    else
                    end if;
                  end if;
                end while;
              end solveOneNonlinearEquation;
            algorithm
              (alpha_min, alpha_max) := findInterval(c1, c2);
              alpha := solveOneNonlinearEquation(c1, c2, alpha_min, alpha_max);
            end normalizationFactor;

            encapsulated function bandPassAlpha "Return alpha for band pass"
              extends Modelica.Icons.Function;
              import Modelica;
              input Real a "Coefficient of s^1";
              input Real b "Coefficient of s^0";
              input Modelica.Units.SI.AngularVelocity w "Bandwidth angular frequency";
              output Real alpha "Alpha factor to build up band pass";
            protected
              Real alpha_min;
              Real alpha_max;
              Real z_min;
              Real z_max;
              Real z;

              function residue "Residue of non-linear equation"
                extends Modelica.Icons.Function;
                input Real a;
                input Real b;
                input Real w;
                input Real z;
                output Real res;
              algorithm
                res := z ^ 2 + (a * w * z / (1 + z)) ^ 2 - (2 + b * w ^ 2) * z + 1;
              end residue;

              function solveOneNonlinearEquation "Solve f(u) = 0; f(u_min) and f(u_max) must have different signs"
                extends Modelica.Icons.Function;
                import Modelica.Utilities.Streams.error;
                input Real aa;
                input Real bb;
                input Real ww;
                input Real u_min "Lower bound of search interval";
                input Real u_max "Upper bound of search interval";
                input Real tolerance = 100 * Modelica.Constants.eps "Relative tolerance of solution u";
                output Real u "Value of independent variable so that f(u) = 0";
              protected
                constant Real eps = Modelica.Constants.eps "Machine epsilon";
                Real a = u_min "Current best minimum interval value";
                Real b = u_max "Current best maximum interval value";
                Real c "Intermediate point a <= c <= b";
                Real d;
                Real e "b - a";
                Real m;
                Real s;
                Real p;
                Real q;
                Real r;
                Real tol;
                Real fa "= f(a)";
                Real fb "= f(b)";
                Real fc;
                Boolean found = false;
              algorithm
                fa := residue(aa, bb, ww, u_min);
                fb := residue(aa, bb, ww, u_max);
                fc := fb;
                if fa > 0.0 and fb > 0.0 or fa < 0.0 and fb < 0.0 then
                  error("The arguments u_min and u_max to solveOneNonlinearEquation(..)\n" + "do not bracket the root of the single non-linear equation:\n" + "  u_min  = " + String(u_min) + "\n" + "  u_max  = " + String(u_max) + "\n" + "  fa = f(u_min) = " + String(fa) + "\n" + "  fb = f(u_max) = " + String(fb) + "\n" + "fa and fb must have opposite sign which is not the case");
                else
                end if;
                c := a;
                fc := fa;
                e := b - a;
                d := e;
                while not found loop
                  if abs(fc) < abs(fb) then
                    a := b;
                    b := c;
                    c := a;
                    fa := fb;
                    fb := fc;
                    fc := fa;
                  else
                  end if;
                  tol := 2 * eps * abs(b) + tolerance;
                  m := (c - b) / 2;
                  if abs(m) <= tol or fb == 0.0 then
                    found := true;
                    u := b;
                  else
                    if abs(e) < tol or abs(fa) <= abs(fb) then
                      e := m;
                      d := e;
                    else
                      s := fb / fa;
                      if a == c then
                        p := 2 * m * s;
                        q := 1 - s;
                      else
                        q := fa / fc;
                        r := fb / fc;
                        p := s * (2 * m * q * (q - r) - (b - a) * (r - 1));
                        q := (q - 1) * (r - 1) * (s - 1);
                      end if;
                      if p > 0 then
                        q := -q;
                      else
                        p := -p;
                      end if;
                      s := e;
                      e := d;
                      if 2 * p < 3 * m * q - abs(tol * q) and p < abs(0.5 * s * q) then
                        d := p / q;
                      else
                        e := m;
                        d := e;
                      end if;
                    end if;
                    a := b;
                    fa := fb;
                    b := b + (if abs(d) > tol then d else if m > 0 then tol else -tol);
                    fb := residue(aa, bb, ww, b);
                    if fb > 0 and fc > 0 or fb < 0 and fc < 0 then
                      c := a;
                      fc := fa;
                      e := b - a;
                      d := e;
                    else
                    end if;
                  end if;
                end while;
              end solveOneNonlinearEquation;
            algorithm
              assert(a ^ 2 / 4 - b <= 0, "Band pass transformation cannot be computed");
              z := solveOneNonlinearEquation(a, b, w, 0, 1);
              alpha := sqrt(z);
            end bandPassAlpha;
          end Utilities;
        end Filter;
      end Internal;
    end Continuous;

    package Interfaces "Library of connectors and partial models for input/output blocks"
      extends Modelica.Icons.InterfacesPackage;
      connector RealInput = input Real "'input Real' as connector";
      connector RealOutput = output Real "'output Real' as connector";

      partial block SO "Single Output continuous control block"
        extends Modelica.Blocks.Icons.Block;
        RealOutput y "Connector of Real output signal";
      end SO;

      partial block SISO "Single Input Single Output continuous control block"
        extends Modelica.Blocks.Icons.Block;
        RealInput u "Connector of Real input signal";
        RealOutput y "Connector of Real output signal";
      end SISO;

      partial block SI2SO "2 Single Input / 1 Single Output continuous control block"
        extends Modelica.Blocks.Icons.Block;
        RealInput u1 "Connector of Real input signal 1";
        RealInput u2 "Connector of Real input signal 2";
        RealOutput y "Connector of Real output signal";
      end SI2SO;

      partial block PartialNoise "Partial noise generator"
        import generator = Modelica.Math.Random.Generators.Xorshift128plus;
        import Modelica.Math.Random.Utilities.automaticLocalSeed;
        extends Modelica.Blocks.Interfaces.SO;
        parameter SI.Period samplePeriod(start = 0.01) "Period for sampling the raw random numbers";
        parameter Boolean enableNoise = globalSeed.enableNoise "= true: y = noise, otherwise y = y_off";
        parameter Real y_off = 0.0 "Sets y = y_off if enableNoise=false (or time<startTime, see below)";
        parameter Boolean useGlobalSeed = true "= true: use global seed, otherwise ignore it";
        parameter Boolean useAutomaticLocalSeed = true "= true: use automatic local seed, otherwise use fixedLocalSeed";
        parameter Integer fixedLocalSeed = 1 "Local seed (any Integer number)";
        parameter SI.Time startTime = 0.0 "Start time for sampling the raw random numbers";
        final parameter Integer localSeed(fixed = false) "The actual localSeed";
      protected
        outer Modelica.Blocks.Noise.GlobalSeed globalSeed "Definition of global seed via inner/outer";
        parameter Integer actualGlobalSeed = if useGlobalSeed then globalSeed.seed else 0 "The global seed, which is actually used";
        parameter Boolean generateNoise = enableNoise and globalSeed.enableNoise "= true, if noise shall be generated, otherwise no noise";
        Integer[generator.nState] state "Internal state of random number generator";
        discrete Real r "Random number according to the desired distribution";
        discrete Real r_raw "Uniform random number in the range (0,1]";
      initial equation
        localSeed = if useAutomaticLocalSeed then automaticLocalSeed(getInstanceName()) else fixedLocalSeed;
        pre(state) = generator.initialState(localSeed, actualGlobalSeed);
        r_raw = generator.random(pre(state));
      equation
        when generateNoise and sample(startTime, samplePeriod) then
          (r_raw, state) = generator.random(pre(state));
        end when;
        y = if not generateNoise or time < startTime then y_off else r;
      end PartialNoise;
    end Interfaces;

    package Math "Library of Real mathematical functions as input/output blocks"
      import Modelica.Blocks.Interfaces;
      extends Modelica.Icons.Package;

      block Add "Output the sum of the two inputs"
        extends Interfaces.SI2SO;
        parameter Real k1 = +1 "Gain of input signal 1";
        parameter Real k2 = +1 "Gain of input signal 2";
      equation
        y = k1 * u1 + k2 * u2;
      end Add;

      block Product "Output product of the two inputs"
        extends Interfaces.SI2SO;
      equation
        y = u1 * u2;
      end Product;
    end Math;

    package Noise "Library of noise blocks"
      extends Modelica.Icons.Package;

      model GlobalSeed "Defines global settings for the blocks of sublibrary Noise, especially a global seed value is defined"
        parameter Boolean enableNoise = true "= true, if noise blocks generate noise as output; = false, if they generate a constant output";
        parameter Boolean useAutomaticSeed = false "= true, choose a seed by system time and process id; = false, use fixedSeed";
        parameter Integer fixedSeed = 67867967 "Fixed global seed for random number generators (if useAutomaticSeed = false)";
        final parameter Integer seed(fixed = false) "Actually used global seed";
        final parameter Integer id_impure(fixed = false) "ID for impure random number generators Modelica.Math.Random.Utilities.impureXXX" annotation(HideResult = true);
      initial equation
        seed = if useAutomaticSeed then Modelica.Math.Random.Utilities.automaticGlobalSeed() else fixedSeed;
        id_impure = Modelica.Math.Random.Utilities.initializeImpureRandom(seed);
        annotation(defaultComponentPrefixes = "inner", missingInnerMessage = "
      Your model is using an outer \"globalSeed\" component but
      an inner \"globalSeed\" component is not defined and therefore
      a default inner \"globalSeed\" component is introduced by the tool.
      To change the default setting, drag Noise.GlobalSeed
      into your model and specify the seed.
      ");
      end GlobalSeed;

      block NormalNoise "Noise generator with normal distribution"
        import distribution = Modelica.Math.Distributions.Normal.quantile;
        extends Modelica.Blocks.Interfaces.PartialNoise;
        parameter Real mu = 0 "Expectation (mean) value of the normal distribution";
        parameter Real sigma(start = 1) "Standard deviation of the normal distribution";
      initial equation
        r = distribution(r_raw, mu, sigma);
      equation
        when generateNoise and sample(startTime, samplePeriod) then
          r = distribution(r_raw, mu, sigma);
        end when;
      end NormalNoise;
    end Noise;

    package Sources "Library of signal source blocks generating Real, Integer and Boolean signals"
      import Modelica.Blocks.Interfaces;
      extends Modelica.Icons.SourcesPackage;

      block RealExpression "Set output signal to a time varying Real expression"
        Modelica.Blocks.Interfaces.RealOutput y = 0.0 "Value of Real output";
      end RealExpression;
    end Sources;

    package Types "Library of constants, external objects and types with choices, especially to build menus"
      extends Modelica.Icons.TypesPackage;
      type Init = enumeration(NoInit "No initialization (start values are used as guess values with fixed=false)", SteadyState "Steady state initialization (derivatives of states are zero)", InitialState "Initialization with initial states", InitialOutput "Initialization with initial outputs (and steady state of the states if possible)") "Enumeration defining initialization of a block" annotation(Evaluate = true);
      type AnalogFilter = enumeration(CriticalDamping "Filter with critical damping", Bessel "Bessel filter", Butterworth "Butterworth filter", ChebyshevI "Chebyshev I filter") "Enumeration defining the method of filtering" annotation(Evaluate = true);
      type FilterType = enumeration(LowPass "Low pass filter", HighPass "High pass filter", BandPass "Band pass filter", BandStop "Band stop / notch filter") "Enumeration of analog filter types (low, high, band pass or band stop filter)" annotation(Evaluate = true);
    end Types;

    package Icons "Icons for Blocks"
      extends Modelica.Icons.IconsPackage;

      partial block Block "Basic graphical layout of input/output block" end Block;
    end Icons;
  end Blocks;

  package Mechanics "Library of 1-dim. and 3-dim. mechanical components (multi-body, rotational, translational)"
    extends Modelica.Icons.Package;
    import Modelica.Units.SI;

    package MultiBody "Library to model 3-dimensional mechanical systems"
      extends Modelica.Icons.Package;
      import Cv = Modelica.Units.Conversions;
      import C = Modelica.Constants;

      model World "World coordinate system + gravity field + default animation definition"
        import Modelica.Mechanics.MultiBody.Types.GravityTypes;
        import Modelica.Mechanics.MultiBody.Types;
        import Modelica.Constants.pi;
        Interfaces.Frame_b frame_b "Coordinate system fixed in the origin of the world frame";
        parameter Boolean enableAnimation = true "= true, if animation of all components is enabled";
        parameter Boolean animateWorld = true "= true, if world coordinate system shall be visualized";
        parameter Boolean animateGravity = true "= true, if gravity field shall be visualized (acceleration vector or field center)";
        parameter Boolean animateGround = false "= true, if ground plane shall be visualized";
        parameter Types.AxisLabel label1 = "x" "Label of horizontal axis in icon";
        parameter Types.AxisLabel label2 = "y" "Label of vertical axis in icon";
        parameter Types.GravityTypes gravityType = GravityTypes.UniformGravity "Type of gravity field" annotation(Evaluate = true);
        parameter SI.Acceleration g = Modelica.Constants.g_n "Constant gravity acceleration";
        parameter Types.Axis n = {0, -1, 0} "Direction of gravity resolved in world frame (gravity = g*n/length(n))" annotation(Evaluate = true);
        parameter Real mu(unit = "m3/s2", min = 0) = 3.986004418e14 "Gravity field constant (default = field constant of earth)";
        parameter Boolean driveTrainMechanics3D = true "= true, if 3-dim. mechanical effects of Parts.Mounting1D/Rotor1D/BevelGear1D shall be taken into account";
        parameter SI.Distance axisLength = nominalLength / 2 "Length of world axes arrows";
        parameter SI.Distance axisDiameter = axisLength / defaultFrameDiameterFraction "Diameter of world axes arrows";
        parameter Boolean axisShowLabels = true "= true, if labels shall be shown";
        input Types.Color axisColor_x = Modelica.Mechanics.MultiBody.Types.Defaults.FrameColor "Color of x-arrow";
        input Types.Color axisColor_y = axisColor_x;
        input Types.Color axisColor_z = axisColor_x "Color of z-arrow";
        parameter SI.Position[3] gravityArrowTail = {0, 0, 0} "Position vector from origin of world frame to arrow tail, resolved in world frame";
        parameter SI.Length gravityArrowLength = axisLength / 2 "Length of gravity arrow";
        parameter SI.Diameter gravityArrowDiameter = gravityArrowLength / defaultWidthFraction "Diameter of gravity arrow";
        input Types.Color gravityArrowColor = {0, 230, 0} "Color of gravity arrow";
        parameter SI.Diameter gravitySphereDiameter = 12742000 "Diameter of sphere representing gravity center (default = mean diameter of earth)";
        input Types.Color gravitySphereColor = {0, 230, 0} "Color of gravity sphere";
        parameter MultiBody.Types.Axis groundAxis_u = if abs(n[1]) >= 0.99 then {0, 1, 0} else {1, 0, 0} "Vector along 1st axis (called u) of ground plane, resolved in world frame (should be perpendicular to gravity direction)";
        parameter SI.Length groundLength_u = 2 "Length of ground plane along groundAxis_u";
        parameter SI.Length groundLength_v = groundLength_u "Length of ground plane perpendicular to groundAxis_u";
        input Types.Color groundColor = {200, 200, 200} "Color of ground plane";
        parameter SI.Length nominalLength = 1 "Nominal length of multi-body system";
        parameter SI.Length defaultAxisLength = nominalLength / 5 "Default for length of a frame axis (but not world frame)";
        parameter SI.Length defaultJointLength = nominalLength / 10 "Default for the fixed length of a shape representing a joint";
        parameter SI.Length defaultJointWidth = nominalLength / 20 "Default for the fixed width of a shape representing a joint";
        parameter SI.Length defaultForceLength = nominalLength / 10 "Default for the fixed length of a shape representing a force (e.g., damper)";
        parameter SI.Length defaultForceWidth = nominalLength / 20 "Default for the fixed width of a shape representing a force (e.g., spring, bushing)";
        parameter SI.Length defaultBodyDiameter = nominalLength / 9 "Default for diameter of sphere representing the center of mass of a body";
        parameter Real defaultWidthFraction = 20 "Default for shape width as a fraction of shape length (e.g., for Parts.FixedTranslation)";
        parameter SI.Length defaultArrowDiameter = nominalLength / 40 "Default for arrow diameter (e.g., of forces, torques, sensors)";
        parameter Real defaultFrameDiameterFraction = 40 "Default for arrow diameter of a coordinate system as a fraction of axis length";
        parameter Real defaultSpecularCoefficient(min = 0) = 0.7 "Default reflection of ambient light (= 0: light is completely absorbed)";
        parameter Real defaultN_to_m(unit = "N/m", min = 0) = 1000 "Default scaling of force arrows (length = force/defaultN_to_m)";
        parameter Real defaultNm_to_m(unit = "N.m/m", min = 0) = 1000 "Default scaling of torque arrows (length = torque/defaultNm_to_m)";
      protected
        parameter Integer ndim = if enableAnimation and animateWorld then 1 else 0;
        parameter Integer ndim2 = if enableAnimation and animateWorld and axisShowLabels then 1 else 0;
        parameter SI.Length headLength = min(axisLength, axisDiameter * Types.Defaults.FrameHeadLengthFraction);
        parameter SI.Length headWidth = axisDiameter * Types.Defaults.FrameHeadWidthFraction;
        parameter SI.Length lineLength = max(0, axisLength - headLength);
        parameter SI.Length lineWidth = axisDiameter;
        parameter SI.Length scaledLabel = Modelica.Mechanics.MultiBody.Types.Defaults.FrameLabelHeightFraction * axisDiameter;
        parameter SI.Length labelStart = 1.05 * axisLength;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape x_arrowLine(shapeType = "cylinder", length = lineLength, width = lineWidth, height = lineWidth, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, color = axisColor_x, specularCoefficient = 0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape x_arrowHead(shapeType = "cone", length = headLength, width = headWidth, height = headWidth, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, color = axisColor_x, r = {lineLength, 0, 0}, specularCoefficient = 0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Internal.Lines x_label(lines = scaledLabel * {[0, 0; 1, 1], [0, 1; 1, 0]}, diameter = axisDiameter, color = axisColor_x, r_lines = {labelStart, 0, 0}, n_x = {1, 0, 0}, n_y = {0, 1, 0}, specularCoefficient = 0) if enableAnimation and animateWorld and axisShowLabels;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape y_arrowLine(shapeType = "cylinder", length = lineLength, width = lineWidth, height = lineWidth, lengthDirection = {0, 1, 0}, widthDirection = {1, 0, 0}, color = axisColor_y, specularCoefficient = 0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape y_arrowHead(shapeType = "cone", length = headLength, width = headWidth, height = headWidth, lengthDirection = {0, 1, 0}, widthDirection = {1, 0, 0}, color = axisColor_y, r = {0, lineLength, 0}, specularCoefficient = 0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Internal.Lines y_label(lines = scaledLabel * {[0, 0; 1, 1.5], [0, 1.5; 0.5, 0.75]}, diameter = axisDiameter, color = axisColor_y, r_lines = {0, labelStart, 0}, n_x = {0, 1, 0}, n_y = {-1, 0, 0}, specularCoefficient = 0) if enableAnimation and animateWorld and axisShowLabels;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape z_arrowLine(shapeType = "cylinder", length = lineLength, width = lineWidth, height = lineWidth, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = axisColor_z, specularCoefficient = 0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape z_arrowHead(shapeType = "cone", length = headLength, width = headWidth, height = headWidth, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = axisColor_z, r = {0, 0, lineLength}, specularCoefficient = 0) if enableAnimation and animateWorld;
        Modelica.Mechanics.MultiBody.Visualizers.Internal.Lines z_label(lines = scaledLabel * {[0, 0; 1, 0], [0, 1; 1, 1], [0, 1; 1, 0]}, diameter = axisDiameter, color = axisColor_z, r_lines = {0, 0, labelStart}, n_x = {0, 0, 1}, n_y = {0, 1, 0}, specularCoefficient = 0) if enableAnimation and animateWorld and axisShowLabels;
        parameter SI.Length gravityHeadLength = min(gravityArrowLength, gravityArrowDiameter * Types.Defaults.ArrowHeadLengthFraction);
        parameter SI.Length gravityHeadWidth = gravityArrowDiameter * Types.Defaults.ArrowHeadWidthFraction;
        parameter SI.Length gravityLineLength = max(0, gravityArrowLength - gravityHeadLength);
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape gravityArrowLine(shapeType = "cylinder", length = gravityLineLength, width = gravityArrowDiameter, height = gravityArrowDiameter, lengthDirection = n, widthDirection = {0, 1, 0}, color = gravityArrowColor, r_shape = gravityArrowTail, specularCoefficient = 0) if enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape gravityArrowHead(shapeType = "cone", length = gravityHeadLength, width = gravityHeadWidth, height = gravityHeadWidth, lengthDirection = n, widthDirection = {0, 1, 0}, color = gravityArrowColor, r_shape = gravityArrowTail + Modelica.Math.Vectors.normalize(n) * gravityLineLength, specularCoefficient = 0) if enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity;
        parameter Integer ndim_pointGravity = if enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity then 1 else 0;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape gravitySphere(shapeType = "sphere", r_shape = {-gravitySphereDiameter / 2, 0, 0}, lengthDirection = {1, 0, 0}, length = gravitySphereDiameter, width = gravitySphereDiameter, height = gravitySphereDiameter, color = gravitySphereColor, specularCoefficient = 0) if enableAnimation and animateGravity and gravityType == GravityTypes.PointGravity;
        Modelica.Mechanics.MultiBody.Visualizers.Advanced.Surface surface(final multiColoredSurface = false, final wireframe = false, final color = groundColor, final specularCoefficient = 0, final transparency = 0, final R = Modelica.Mechanics.MultiBody.Frames.absoluteRotation(Modelica.Mechanics.MultiBody.Frames.from_nxy(n, groundAxis_u), Modelica.Mechanics.MultiBody.Frames.axesRotations({1, 2, 3}, {pi / 2, pi / 2, 0}, {0, 0, 0})), final r_0 = zeros(3), final nu = 2, final nv = 2, redeclare function surfaceCharacteristic = Modelica.Mechanics.MultiBody.Visualizers.Advanced.SurfaceCharacteristics.rectangle(lu = groundLength_u, lv = groundLength_v)) if enableAnimation and animateGround and gravityType == GravityTypes.UniformGravity;
      equation
        Connections.root(frame_b.R);
        assert(Modelica.Math.Vectors.length(n) > 1e-10, "Parameter n of World object is wrong (length(n) > 0 required)");
        frame_b.r_0 = zeros(3);
        frame_b.R = Frames.nullRotation();
        annotation(defaultComponentPrefixes = "inner", missingInnerMessage = "No \"world\" component is defined. A default world
      component with the default gravity field will be used
      (g=9.81 in negative y-axis). If this is not desired,
      drag Modelica.Mechanics.MultiBody.World into the top level of your model.");
      end World;

      package Frames "Functions to transform rotational frame quantities"
        extends Modelica.Icons.Package;

        record Orientation "Orientation object defining rotation from a frame 1 into a frame 2"
          extends Modelica.Icons.Record;
          Real[3, 3] T "Transformation matrix from world frame to local frame";
          SI.AngularVelocity[3] w "Absolute angular velocity of local frame, resolved in local frame";

          encapsulated function equalityConstraint "Return the constraint residues to express that two frames have the same orientation"
            import Modelica;
            import Modelica.Mechanics.MultiBody.Frames;
            extends Modelica.Icons.Function;
            input Frames.Orientation R1 "Orientation object to rotate frame 0 into frame 1";
            input Frames.Orientation R2 "Orientation object to rotate frame 0 into frame 2";
            output Real[3] residue "The rotation angles around x-, y-, and z-axis of frame 1 to rotate frame 1 into frame 2 for a small rotation (should be zero)";
          algorithm
            residue := {Modelica.Math.atan2(cross(R1.T[1, :], R1.T[2, :]) * R2.T[2, :], R1.T[1, :] * R2.T[1, :]), Modelica.Math.atan2(-cross(R1.T[1, :], R1.T[2, :]) * R2.T[1, :], R1.T[2, :] * R2.T[2, :]), Modelica.Math.atan2(R1.T[2, :] * R2.T[1, :], R1.T[3, :] * R2.T[3, :])};
            annotation(Inline = true);
          end equalityConstraint;
        end Orientation;

        function resolve1 "Transform vector from frame 2 to frame 1"
          extends Modelica.Icons.Function;
          input Orientation R "Orientation object to rotate frame 1 into frame 2";
          input Real[3] v2 "Vector in frame 2";
          output Real[3] v1 "Vector in frame 1";
        algorithm
          v1 := transpose(R.T) * v2;
          annotation(derivative(noDerivative = R) = Internal.resolve1_der, InlineAfterIndexReduction = true);
        end resolve1;

        function resolve2 "Transform vector from frame 1 to frame 2"
          extends Modelica.Icons.Function;
          input Orientation R "Orientation object to rotate frame 1 into frame 2";
          input Real[3] v1 "Vector in frame 1";
          output Real[3] v2 "Vector in frame 2";
        algorithm
          v2 := R.T * v1;
          annotation(derivative(noDerivative = R) = Internal.resolve2_der, InlineAfterIndexReduction = true);
        end resolve2;

        function nullRotation "Return orientation object that does not rotate a frame"
          extends Modelica.Icons.Function;
          output Orientation R "Orientation object such that frame 1 and frame 2 are identical";
        algorithm
          R := Orientation(T = identity(3), w = zeros(3));
          annotation(Inline = true);
        end nullRotation;

        function absoluteRotation "Return absolute orientation object from another absolute and a relative orientation object"
          extends Modelica.Icons.Function;
          input Orientation R1 "Orientation object to rotate frame 0 into frame 1";
          input Orientation R_rel "Orientation object to rotate frame 1 into frame 2";
          output Orientation R2 "Orientation object to rotate frame 0 into frame 2";
        algorithm
          R2 := Orientation(T = R_rel.T * R1.T, w = resolve2(R_rel, R1.w) + R_rel.w);
          annotation(Inline = true);
        end absoluteRotation;

        function axesRotations "Return fixed rotation object to rotate in sequence around fixed angles along 3 axes"
          import TM = Modelica.Mechanics.MultiBody.Frames.TransformationMatrices;
          extends Modelica.Icons.Function;
          input Integer[3] sequence(min = {1, 1, 1}, max = {3, 3, 3}) = {1, 2, 3} "Sequence of rotations from frame 1 to frame 2 along axis sequence[i]";
          input SI.Angle[3] angles "Rotation angles around the axes defined in 'sequence'";
          input SI.AngularVelocity[3] der_angles "= der(angles)";
          output Orientation R "Orientation object to rotate frame 1 into frame 2";
        algorithm
          R := Orientation(T = TM.axisRotation(sequence[3], angles[3]) * TM.axisRotation(sequence[2], angles[2]) * TM.axisRotation(sequence[1], angles[1]), w = Frames.axis(sequence[3]) * der_angles[3] + TM.resolve2(TM.axisRotation(sequence[3], angles[3]), Frames.axis(sequence[2]) * der_angles[2]) + TM.resolve2(TM.axisRotation(sequence[3], angles[3]) * TM.axisRotation(sequence[2], angles[2]), Frames.axis(sequence[1]) * der_angles[1]));
          annotation(Inline = true);
        end axesRotations;

        function from_nxy "Return fixed orientation object from n_x and n_y vectors"
          extends Modelica.Icons.Function;
          input Real[3] n_x(each final unit = "1") "Vector in direction of x-axis of frame 2, resolved in frame 1";
          input Real[3] n_y(each final unit = "1") "Vector in direction of y-axis of frame 2, resolved in frame 1";
          output Orientation R "Orientation object to rotate frame 1 into frame 2";
        algorithm
          R := Orientation(T = TransformationMatrices.from_nxy(n_x, n_y), w = zeros(3));
        end from_nxy;

        function axis "Return unit vector for x-, y-, or z-axis"
          extends Modelica.Icons.Function;
          input Integer axis(min = 1, max = 3) "Axis vector to be returned";
          output Real[3] e(each final unit = "1") "Unit axis vector";
        algorithm
          e := if axis == 1 then {1, 0, 0} else if axis == 2 then {0, 1, 0} else {0, 0, 1};
          annotation(Inline = true);
        end axis;

        package TransformationMatrices "Functions for transformation matrices"
          extends Modelica.Icons.FunctionsPackage;

          type Orientation "Orientation type defining rotation from a frame 1 into a frame 2 with a transformation matrix"
            extends Internal.TransformationMatrix;

            encapsulated function equalityConstraint "Return the constraint residues to express that two frames have the same orientation"
              import Modelica;
              import Modelica.Mechanics.MultiBody.Frames.TransformationMatrices;
              extends Modelica.Icons.Function;
              input TransformationMatrices.Orientation T1 "Orientation object to rotate frame 0 into frame 1";
              input TransformationMatrices.Orientation T2 "Orientation object to rotate frame 0 into frame 2";
              output Real[3] residue "The rotation angles around x-, y-, and z-axis of frame 1 to rotate frame 1 into frame 2 for a small rotation (should be zero)";
            algorithm
              residue := {cross(T1[1, :], T1[2, :]) * T2[2, :], -cross(T1[1, :], T1[2, :]) * T2[1, :], T1[2, :] * T2[1, :]};
              annotation(Inline = true);
            end equalityConstraint;
          end Orientation;

          function resolve1 "Transform vector from frame 2 to frame 1"
            extends Modelica.Icons.Function;
            input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
            input Real[3] v2 "Vector in frame 2";
            output Real[3] v1 "Vector in frame 1";
          algorithm
            v1 := transpose(T) * v2;
            annotation(Inline = true);
          end resolve1;

          function resolve2 "Transform vector from frame 1 to frame 2"
            extends Modelica.Icons.Function;
            input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
            input Real[3] v1 "Vector in frame 1";
            output Real[3] v2 "Vector in frame 2";
          algorithm
            v2 := T * v1;
            annotation(Inline = true);
          end resolve2;

          function absoluteRotation "Return absolute orientation object from another absolute and a relative orientation object"
            extends Modelica.Icons.Function;
            input TransformationMatrices.Orientation T1 "Orientation object to rotate frame 0 into frame 1";
            input TransformationMatrices.Orientation T_rel "Orientation object to rotate frame 1 into frame 2";
            output TransformationMatrices.Orientation T2 "Orientation object to rotate frame 0 into frame 2";
          algorithm
            T2 := T_rel * T1;
            annotation(Inline = true);
          end absoluteRotation;

          function axisRotation "Return rotation object to rotate around one frame axis"
            extends Modelica.Icons.Function;
            input Integer axis(min = 1, max = 3) "Rotate around 'axis' of frame 1";
            input SI.Angle angle "Rotation angle to rotate frame 1 into frame 2 along 'axis' of frame 1";
            output TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
          algorithm
            T := if axis == 1 then [1, 0, 0; 0, Modelica.Math.cos(angle), Modelica.Math.sin(angle); 0, -Modelica.Math.sin(angle), Modelica.Math.cos(angle)] else if axis == 2 then [Modelica.Math.cos(angle), 0, -Modelica.Math.sin(angle); 0, 1, 0; Modelica.Math.sin(angle), 0, Modelica.Math.cos(angle)] else [Modelica.Math.cos(angle), Modelica.Math.sin(angle), 0; -Modelica.Math.sin(angle), Modelica.Math.cos(angle), 0; 0, 0, 1];
            annotation(Inline = true);
          end axisRotation;

          function from_nxy "Return orientation object from n_x and n_y vectors"
            extends Modelica.Icons.Function;
            import Modelica.Math.Vectors.length;
            import Modelica.Math.Vectors.normalize;
            input Real[3] n_x(each final unit = "1") "Vector in direction of x-axis of frame 2, resolved in frame 1";
            input Real[3] n_y(each final unit = "1") "Vector in direction of y-axis of frame 2, resolved in frame 1";
            output TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
          protected
            Real[3] e_x(each final unit = "1") = if length(n_x) < 1e-10 then {1, 0, 0} else normalize(n_x);
            Real[3] e_y(each final unit = "1") = if length(n_y) < 1e-10 then {0, 1, 0} else normalize(n_y);
            Real[3] n_z_aux(each final unit = "1") = cross(e_x, e_y);
            Real[3] n_y_aux(each final unit = "1") = if n_z_aux * n_z_aux > 1.0e-6 then e_y else if abs(e_x[1]) > 1.0e-6 then {0, 1, 0} else {1, 0, 0};
            Real[3] e_z_aux(each final unit = "1") = cross(e_x, n_y_aux);
            Real[3] e_z(each final unit = "1") = normalize(e_z_aux);
          algorithm
            T := {e_x, cross(e_z, e_x), e_z};
          end from_nxy;
        end TransformationMatrices;

        package Internal "Internal definitions that may be removed or changed (do not use)"
          extends Modelica.Icons.InternalPackage;
          type TransformationMatrix = Real[3, 3];

          function resolve1_der "Derivative of function Frames.resolve1(..)"
            import Modelica.Mechanics.MultiBody.Frames;
            extends Modelica.Icons.Function;
            input Orientation R "Orientation object to rotate frame 1 into frame 2";
            input Real[3] v2 "Vector resolved in frame 2";
            input Real[3] v2_der "= der(v2)";
            output Real[3] v1_der "Derivative of vector v resolved in frame 1";
          algorithm
            v1_der := Frames.resolve1(R, v2_der + cross(R.w, v2));
            annotation(Inline = true);
          end resolve1_der;

          function resolve2_der "Derivative of function Frames.resolve2(..)"
            import Modelica.Mechanics.MultiBody.Frames;
            extends Modelica.Icons.Function;
            input Orientation R "Orientation object to rotate frame 1 into frame 2";
            input Real[3] v1 "Vector resolved in frame 1";
            input Real[3] v1_der "= der(v1)";
            output Real[3] v2_der "Derivative of vector v resolved in frame 2";
          algorithm
            v2_der := Frames.resolve2(R, v1_der) - cross(R.w, Frames.resolve2(R, v1));
            annotation(Inline = true);
          end resolve2_der;
        end Internal;
      end Frames;

      package Interfaces "Connectors and partial models for 3-dim. mechanical components"
        extends Modelica.Icons.InterfacesPackage;

        connector Frame "Coordinate system fixed to the component with one cut-force and cut-torque (no icon)"
          SI.Position[3] r_0 "Position vector from world frame to the connector frame origin, resolved in world frame";
          Frames.Orientation R "Orientation object to rotate the world frame into the connector frame";
          flow SI.Force[3] f "Cut-force resolved in connector frame" annotation(unassignedMessage = "All Forces cannot be uniquely calculated.
        The reason could be that the mechanism contains
        a planar loop or that joints constrain the
        same motion. For planar loops, use for one
        revolute joint per loop the joint
        Joints.RevolutePlanarLoopConstraint instead of
        Joints.Revolute.");
          flow SI.Torque[3] t "Cut-torque resolved in connector frame";
        end Frame;

        connector Frame_a "Coordinate system fixed to the component with one cut-force and cut-torque (filled rectangular icon)"
          extends Frame;
        end Frame_a;

        connector Frame_b "Coordinate system fixed to the component with one cut-force and cut-torque (non-filled rectangular icon)"
          extends Frame;
        end Frame_b;

        partial model PartialElementaryJoint "Base model for elementary joints (has two frames + outer world + assert to guarantee that the joint is connected)"
          Interfaces.Frame_a frame_a "Coordinate system fixed to the joint with one cut-force and cut-torque";
          Interfaces.Frame_b frame_b "Coordinate system fixed to the joint with one cut-force and cut-torque";
        protected
          outer Modelica.Mechanics.MultiBody.World world;
        equation
          Connections.branch(frame_a.R, frame_b.R);
          assert(cardinality(frame_a) > 0, "Connector frame_a of joint object is not connected");
          assert(cardinality(frame_b) > 0, "Connector frame_b of joint object is not connected");
        end PartialElementaryJoint;

        partial function partialSurfaceCharacteristic "Interface for a function returning surface characteristics"
          extends Modelica.Icons.Function;
          input Integer nu "Number of points in u-Dimension";
          input Integer nv "Number of points in v-Dimension";
          input Boolean multiColoredSurface = false "= true: Color is defined for each surface point";
          output SI.Position[nu, nv] X "[nu,nv] positions of points in x-Direction resolved in surface frame";
          output SI.Position[nu, nv] Y "[nu,nv] positions of points in y-Direction resolved in surface frame";
          output SI.Position[nu, nv] Z "[nu,nv] positions of points in z-Direction resolved in surface frame";
          output Real[if multiColoredSurface then nu else 0, if multiColoredSurface then nv else 0, 3] C "[nu,nv,3] Color array, defining the color for each surface point";
        end partialSurfaceCharacteristic;
      end Interfaces;

      package Joints "Components that constrain the motion between two frames"
        extends Modelica.Icons.Package;

        model Prismatic "Prismatic joint (1 translational degree-of-freedom, 2 potential states, optional axis flange)"
          extends Modelica.Mechanics.MultiBody.Interfaces.PartialElementaryJoint;
          Modelica.Mechanics.Translational.Interfaces.Flange_a axis if useAxisFlange "1-dim. translational flange that drives the joint";
          Modelica.Mechanics.Translational.Interfaces.Flange_b support if useAxisFlange "1-dim. translational flange of the drive support (assumed to be fixed in the world frame, NOT in the joint)";
          parameter Boolean useAxisFlange = false "= true, if axis flange is enabled" annotation(Evaluate = true, HideResult = true);
          parameter Boolean animation = true "= true, if animation shall be enabled";
          parameter Modelica.Mechanics.MultiBody.Types.Axis n = {1, 0, 0} "Axis of translation resolved in frame_a (= same as in frame_b)" annotation(Evaluate = true);
          parameter Types.Axis boxWidthDirection = {0, 1, 0} "Vector in width direction of box, resolved in frame_a" annotation(Evaluate = true);
          parameter SI.Distance boxWidth = world.defaultJointWidth "Width of prismatic joint box";
          parameter SI.Distance boxHeight = boxWidth "Height of prismatic joint box";
          input Types.Color boxColor = Modelica.Mechanics.MultiBody.Types.Defaults.JointColor "Color of prismatic joint box";
          input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)";
          parameter StateSelect stateSelect = StateSelect.prefer "Priority to use distance s and v=der(s) as states";
          final parameter Real[3] e(each final unit = "1") = Modelica.Math.Vectors.normalizeWithAssert(n) "Unit vector in direction of prismatic axis n";
          SI.Position s(start = 0, final stateSelect = stateSelect) "Relative distance between frame_a and frame_b" annotation(unassignedMessage = "
        The relative distance s of a prismatic joint cannot be determined.
        Possible reasons:
        - A non-zero mass might be missing on either side of the parts
          connected to the prismatic joint.
        - Too many StateSelect.always are defined and the model
          has less degrees of freedom as specified with this setting
          (remove all StateSelect.always settings).
          ");
          SI.Velocity v(start = 0, final stateSelect = stateSelect) "First derivative of s (relative velocity)";
          SI.Acceleration a(start = 0) "Second derivative of s (relative acceleration)";
          SI.Force f "Actuation force in direction of joint axis";
        protected
          Visualizers.Advanced.Shape box(shapeType = "box", color = boxColor, specularCoefficient = specularCoefficient, length = if noEvent(abs(s) > 1.e-6) then s else 1.e-6, width = boxWidth, height = boxHeight, lengthDirection = e, widthDirection = boxWidthDirection, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
          Translational.Components.Fixed fixed;
          Translational.Interfaces.InternalSupport internalAxis(f = f);
          Translational.Sources.ConstantForce constantForce(f_constant = 0) if not useAxisFlange;
        equation
          v = der(s);
          a = der(v);
          frame_b.r_0 = frame_a.r_0 + Frames.resolve1(frame_a.R, e * s);
          frame_b.R = frame_a.R;
          zeros(3) = frame_a.f + frame_b.f;
          zeros(3) = frame_a.t + frame_b.t + cross(e * s, frame_b.f);
          f = -e * frame_b.f;
          s = internalAxis.s;
          connect(fixed.flange, support);
          connect(internalAxis.flange, axis);
          connect(constantForce.flange, internalAxis.flange);
        end Prismatic;
      end Joints;

      package Parts "Rigid components such as bodies with mass and inertia and massless rods"
        extends Modelica.Icons.Package;

        model FixedTranslation "Fixed translation of frame_b with respect to frame_a"
          import Modelica.Mechanics.MultiBody.Types;
          import Modelica.Units.Conversions.to_unit1;
          Interfaces.Frame_a frame_a "Coordinate system fixed to the component with one cut-force and cut-torque";
          Interfaces.Frame_b frame_b "Coordinate system fixed to the component with one cut-force and cut-torque";
          parameter Boolean animation = true "= true, if animation shall be enabled";
          parameter SI.Position[3] r(start = {0, 0, 0}) "Vector from frame_a to frame_b resolved in frame_a";
          parameter Types.ShapeType shapeType = "cylinder" "Type of shape";
          parameter SI.Position[3] r_shape = {0, 0, 0} "Vector from frame_a to shape origin, resolved in frame_a";
          parameter Types.Axis lengthDirection = to_unit1(r - r_shape) "Vector in length direction of shape, resolved in frame_a" annotation(Evaluate = true);
          parameter Types.Axis widthDirection = {0, 1, 0} "Vector in width direction of shape, resolved in frame_a" annotation(Evaluate = true);
          parameter SI.Length length = Modelica.Math.Vectors.length(r - r_shape) "Length of shape";
          parameter SI.Distance width = length / world.defaultWidthFraction "Width of shape";
          parameter SI.Distance height = width "Height of shape";
          parameter Types.ShapeExtra extra = 0.0 "Additional parameter depending on shapeType (see docu of Visualizers.Advanced.Shape)";
          input Types.Color color = Modelica.Mechanics.MultiBody.Types.Defaults.RodColor "Color of shape";
          input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)";
        protected
          outer Modelica.Mechanics.MultiBody.World world;
          Visualizers.Advanced.Shape shape(shapeType = shapeType, color = color, specularCoefficient = specularCoefficient, r_shape = r_shape, lengthDirection = lengthDirection, widthDirection = widthDirection, length = length, width = width, height = height, extra = extra, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
        equation
          Connections.branch(frame_a.R, frame_b.R);
          assert(cardinality(frame_a) > 0 or cardinality(frame_b) > 0, "Neither connector frame_a nor frame_b of FixedTranslation object is connected");
          frame_b.r_0 = frame_a.r_0 + Frames.resolve1(frame_a.R, r);
          frame_b.R = frame_a.R;
          zeros(3) = frame_a.f + frame_b.f;
          zeros(3) = frame_a.t + frame_b.t + cross(r, frame_b.f);
        end FixedTranslation;
      end Parts;

      package Visualizers "3-dimensional visual objects used for animation"
        extends Modelica.Icons.Package;

        package Advanced "Visualizers that require basic knowledge about Modelica in order to use them"
          extends Modelica.Icons.Package;

          model Shape "Visualizing an elementary object with variable size; all data have to be set as modifiers (see info layer)"
            extends ModelicaServices.Animation.Shape;
            extends Modelica.Utilities.Internal.PartialModelicaServices.Animation.PartialShape;
          end Shape;

          model Surface "Visualizing a moveable, parameterized surface; the surface characteristic is provided by a function"
            extends Modelica.Mechanics.MultiBody.Icons.Surface;
            extends Modelica.Utilities.Internal.PartialModelicaServices.Animation.PartialSurface;
            extends ModelicaServices.Animation.Surface;
          end Surface;

          package SurfaceCharacteristics "Functions returning surface descriptions"
            extends Modelica.Icons.FunctionsPackage;

            function rectangle "Function defining the surface characteristic of a planar rectangle"
              extends Modelica.Mechanics.MultiBody.Interfaces.partialSurfaceCharacteristic(final multiColoredSurface = false);
              input SI.Distance lu = 1 "Length in direction u";
              input SI.Distance lv = 3 "Length in direction v";
            algorithm
              X[:, :] := lu / 2 * transpose(fill(linspace(-1, 1, nu), nv));
              Y[:, :] := lv / 2 * fill(linspace(-1, 1, nv), nu);
              Z[:, :] := fill(0, nu, nv);
            end rectangle;
          end SurfaceCharacteristics;
        end Advanced;

        package Internal "Makeshift visualizers for display of three-dimensional fonts using cylinders"
          extends Modelica.Icons.InternalPackage;

          model Lines "Visualizing a set of lines as cylinders with variable size, e.g., used to display characters (no Frame connector)"
            import Modelica.Mechanics.MultiBody;
            import Modelica.Mechanics.MultiBody.Types;
            import Modelica.Mechanics.MultiBody.Frames;
            import T = Modelica.Mechanics.MultiBody.Frames.TransformationMatrices;
            input Modelica.Mechanics.MultiBody.Frames.Orientation R = Frames.nullRotation() "Orientation object to rotate the world frame into the object frame";
            input SI.Position[3] r = {0, 0, 0} "Position vector from origin of world frame to origin of object frame, resolved in world frame";
            input SI.Position[3] r_lines = {0, 0, 0} "Position vector from origin of object frame to the origin of 'lines' frame, resolved in object frame";
            input Real[3] n_x(each final unit = "1") = {1, 0, 0} "Vector in direction of x-axis of 'lines' frame, resolved in object frame";
            input Real[3] n_y(each final unit = "1") = {0, 1, 0} "Vector in direction of y-axis of 'lines' frame, resolved in object frame";
            input SI.Position[:, 2, 2] lines = zeros(0, 2, 2) "List of start and end points of cylinders resolved in an x-y frame defined by n_x, n_y, e.g., {[0,0;1,1], [0,1;1,0], [2,0; 3,1]}";
            input SI.Length diameter(min = 0) = 0.05 "Diameter of the cylinders defined by lines";
            input Modelica.Mechanics.MultiBody.Types.Color color = {0, 128, 255} "Color of cylinders";
            input Types.SpecularCoefficient specularCoefficient = 0.7 "Reflection of ambient light (= 0: light is completely absorbed)";
          protected
            parameter Integer n = size(lines, 1) "Number of cylinders";
            T.Orientation R_rel = T.from_nxy(n_x, n_y);
            T.Orientation R_lines = T.absoluteRotation(R.T, R_rel);
            SI.Position[3] r_abs = r + T.resolve1(R.T, r_lines);
            Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape[n] cylinders(each shapeType = "cylinder", lengthDirection = {T.resolve1(R_rel, vector([lines[i, 2, :] - lines[i, 1, :]; 0])) for i in 1:n}, length = {Modelica.Math.Vectors.length(lines[i, 2, :] - lines[i, 1, :]) for i in 1:n}, r = {r_abs + T.resolve1(R_lines, vector([lines[i, 1, :]; 0])) for i in 1:n}, each width = diameter, each height = diameter, each widthDirection = {0, 1, 0}, each color = color, each R = R, each specularCoefficient = specularCoefficient);
          end Lines;
        end Internal;
      end Visualizers;

      package Types "Constants and types with choices, especially to build menus"
        extends Modelica.Icons.TypesPackage;
        type Axis = Modelica.Icons.TypeReal[3](each final unit = "1") "Axis vector with choices" annotation(Evaluate = true);
        type AxisLabel = Modelica.Icons.TypeString "Label of axis with choices";
        type Color = Modelica.Icons.TypeInteger[3](each min = 0, each max = 255) "RGB representation of color";
        type SpecularCoefficient = Modelica.Icons.TypeReal(min = 0) "Reflection of ambient light (= 0: light is completely absorbed)";
        type ShapeType = Modelica.Icons.TypeString "Type of shape (box, sphere, cylinder, pipecylinder, cone, pipe, beam, gearwheel, spring, <external shape>)";
        type ShapeExtra = Modelica.Icons.TypeReal "Type of the additional data that can be defined for an elementary ShapeType";
        type GravityTypes = enumeration(NoGravity "No gravity field", UniformGravity "Uniform gravity field", PointGravity "Point gravity field") "Enumeration defining the type of the gravity field";

        package Defaults "Default settings of the MultiBody library via constants"
          extends Modelica.Icons.Package;
          constant Types.Color RodColor = {155, 155, 155} "Default color for massless rod shapes (grey)";
          constant Types.Color JointColor = {255, 0, 0} "Default color for elementary joints (red)";
          constant Types.Color FrameColor = {0, 0, 0} "Default color for frame axes and labels (black)";
          constant Real FrameHeadLengthFraction = 5.0 "Frame arrow head length / arrow diameter";
          constant Real FrameHeadWidthFraction = 3.0 "Frame arrow head width / arrow diameter";
          constant Real FrameLabelHeightFraction = 3.0 "Height of frame label / arrow diameter";
          constant Real ArrowHeadLengthFraction = 4.0 "Arrow head length / arrow diameter";
          constant Real ArrowHeadWidthFraction = 3.0 "Arrow head width / arrow diameter";
        end Defaults;
      end Types;

      package Icons "Icons for MultiBody package"
        extends Modelica.Icons.IconsPackage;

        model Surface "Surface icon" end Surface;
      end Icons;
    end MultiBody;

    package Translational "Library to model 1-dimensional, translational mechanical systems"
      extends Modelica.Icons.Package;

      package Components "Components for 1D translational mechanical drive trains"
        extends Modelica.Icons.Package;

        model Fixed "Fixed flange"
          parameter SI.Position s0 = 0 "Fixed offset position of housing";
          Interfaces.Flange_b flange;
        equation
          flange.s = s0;
        end Fixed;
      end Components;

      package Sources "Sources to drive 1D translational mechanical components"
        extends Modelica.Icons.SourcesPackage;

        model Position "Forced movement of a flange according to a reference position"
          extends Modelica.Mechanics.Translational.Interfaces.PartialElementaryOneFlangeAndSupport2(s(stateSelect = if exact then StateSelect.default else StateSelect.prefer));
          parameter Boolean exact = false "Is true/false for exact treatment/filtering of the input signal, respectively" annotation(Evaluate = true);
          parameter SI.Frequency f_crit = 50 "If exact=false, critical frequency of filter to filter input signal";
          SI.Velocity v(start = 0, stateSelect = if exact then StateSelect.default else StateSelect.prefer) "If exact=false, absolute velocity of flange else dummy";
          SI.Acceleration a(start = 0) "If exact=false, absolute acceleration of flange else dummy";
          Modelica.Blocks.Interfaces.RealInput s_ref(unit = "m") "Reference position of flange as input signal";
        protected
          parameter SI.AngularFrequency w_crit = 2 * Modelica.Constants.pi * f_crit "Critical frequency";
          constant Real af = 1.3617 "s coefficient of Bessel filter";
          constant Real bf = 0.6180 "s*s coefficient of Bessel filter";
        initial equation
          if not exact then
            s = s_ref;
          end if;
        equation
          if exact then
            s = s_ref;
            v = 0;
            a = 0;
          else
            v = der(s);
            a = der(v);
            a = ((s_ref - s) * w_crit - af * v) * (w_crit / bf);
          end if;
        end Position;

        model ConstantForce "Constant force, not dependent on speed"
          extends Modelica.Mechanics.Translational.Interfaces.PartialForce;
          parameter SI.Force f_constant "Nominal force (if negative, force is acting as load in positive direction of motion)";
        equation
          f = -f_constant;
        end ConstantForce;
      end Sources;

      package Interfaces "Interfaces for 1-dim. translational mechanical components"
        extends Modelica.Icons.InterfacesPackage;

        connector Flange "One-dimensional translational flange"
          SI.Position s "Absolute position of flange";
          flow SI.Force f "Cut force directed into flange";
        end Flange;

        connector Flange_a "One-dimensional translational flange (left, flange axis directed INTO cut plane)"
          extends Flange;
        end Flange_a;

        connector Flange_b "One-dimensional translational flange (right, flange axis directed OUT OF cut plane)"
          extends Flange;
        end Flange_b;

        connector Support "Support/housing flange of a one-dimensional translational component"
          extends Flange;
        end Support;

        model InternalSupport "Adapter model to utilize conditional support connector"
          input SI.Force f "External support force (must be computed via force balance in model where InternalSupport is used; = flange.f)";
          SI.Position s "External support position (= flange.s)";
          Flange_a flange "Internal support flange (must be connected to the conditional support connector for useSupport=true and to conditional fixed model for useSupport=false)";
        equation
          flange.f = f;
          flange.s = s;
        end InternalSupport;

        partial model PartialElementaryOneFlangeAndSupport2 "Partial model for a component with one translational 1-dim. shaft flange and a support used for textual modeling, i.e., for elementary models"
          parameter Boolean useSupport = false "= true, if support flange enabled, otherwise implicitly grounded" annotation(Evaluate = true, HideResult = true);
          SI.Length s "Distance between flange and support (= flange.s - support.s)";
          Flange_b flange "Flange of component";
          Support support(s = s_support, f = -flange.f) if useSupport "Support/housing of component";
        protected
          SI.Length s_support "Absolute position of support flange";
        equation
          s = flange.s - s_support;
          if not useSupport then
            s_support = 0;
          end if;
        end PartialElementaryOneFlangeAndSupport2;

        partial model PartialForce "Partial model of a force acting at the flange (accelerates the flange)"
          extends PartialElementaryOneFlangeAndSupport2;
          SI.Force f "Accelerating force acting at flange (= flange.f)";
        equation
          f = flange.f;
        end PartialForce;
      end Interfaces;
    end Translational;
  end Mechanics;

  package Math "Library of mathematical functions (e.g., sin, cos) and of functions operating on vectors and matrices"
    extends Modelica.Icons.Package;

    package Vectors "Library of functions operating on vectors"
      extends Modelica.Icons.Package;

      function length "Return length of a vector (better as norm(), if further symbolic processing is performed)"
        extends Modelica.Icons.Function;
        input Real[:] v "Real vector";
        output Real result "Length of vector v";
      algorithm
        result := sqrt(v * v);
        annotation(Inline = true);
      end length;

      function normalize "Return normalized vector such that length = 1 and prevent zero-division for zero vector"
        extends Modelica.Icons.Function;
        input Real[:] v "Real vector";
        input Real eps(min = 0.0) = 100 * Modelica.Constants.eps "if |v| < eps then result = v/eps";
        output Real[size(v, 1)] result "Input vector v normalized to length=1";
      algorithm
        result := smooth(0, if length(v) >= eps then v / length(v) else v / eps);
        annotation(Inline = true);
      end normalize;

      function normalizeWithAssert "Return normalized vector such that length = 1 (trigger an assert for zero vector)"
        import Modelica.Math.Vectors.length;
        extends Modelica.Icons.Function;
        input Real[:] v "Real vector";
        output Real[size(v, 1)] result "Input vector v normalized to length=1";
      algorithm
        assert(length(v) > 0.0, "Vector v={0,0,0} shall be normalized (= v/sqrt(v*v)), but this results in a division by zero.\nProvide a non-zero vector!");
        result := v / length(v);
        annotation(Inline = true);
      end normalizeWithAssert;
    end Vectors;

    package Random "Library of functions for generating random numbers"
      extends Modelica.Icons.Package;

      package Generators "Library of functions generating uniform random numbers in the range 0 < random <= 1.0 (with exposed state vectors)"
        extends Modelica.Icons.Package;

        package Xorshift64star "Random number generator xorshift64*"
          constant Integer nState = 2 "The dimension of the internal state vector";
          extends Modelica.Icons.Package;

          function initialState "Returns an initial state for the xorshift64* algorithm"
            extends Modelica.Icons.Function;
            input Integer localSeed "The local seed to be used for generating initial states";
            input Integer globalSeed "The global seed to be combined with the local seed";
            output Integer[nState] state "The generated initial states";
          protected
            Real r "Random number not used outside the function";
            constant Integer p = 10 "The number of iterations to use";
          algorithm
            if localSeed == 0 and globalSeed == 0 then
              state := {126247697, globalSeed};
            else
              state := {localSeed, globalSeed};
            end if;
            for i in 1:p loop
              (r, state) := random(state);
            end for;
          end initialState;

          function random "Returns a uniform random number with the xorshift64* algorithm"
            extends Modelica.Icons.Function;
            input Integer[nState] stateIn "The internal states for the random number generator";
            output Real result "A random number with a uniform distribution on the interval (0,1]";
            output Integer[nState] stateOut "The new internal states of the random number generator";
            external "C" ModelicaRandom_xorshift64star(stateIn, stateOut, result) annotation(IncludeDirectory = "modelica://Modelica/Resources/C-Sources", Library = "ModelicaExternalC");
          end random;
        end Xorshift64star;

        package Xorshift128plus "Random number generator xorshift128+"
          constant Integer nState = 4 "The dimension of the internal state vector";
          extends Modelica.Icons.Package;

          function initialState "Returns an initial state for the xorshift128+ algorithm"
            extends Modelica.Icons.Function;
            input Integer localSeed "The local seed to be used for generating initial states";
            input Integer globalSeed "The global seed to be combined with the local seed";
            output Integer[nState] state "The generated initial states";
          algorithm
            state := Utilities.initialStateWithXorshift64star(localSeed, globalSeed, size(state, 1));
            annotation(Inline = true);
          end initialState;

          function random "Returns a uniform random number with the xorshift128+ algorithm"
            extends Modelica.Icons.Function;
            input Integer[nState] stateIn "The internal states for the random number generator";
            output Real result "A random number with a uniform distribution on the interval (0,1]";
            output Integer[nState] stateOut "The new internal states of the random number generator";
            external "C" ModelicaRandom_xorshift128plus(stateIn, stateOut, result) annotation(IncludeDirectory = "modelica://Modelica/Resources/C-Sources", Library = "ModelicaExternalC");
          end random;
        end Xorshift128plus;
      end Generators;

      package Utilities "Library of utility functions for the Random package (usually of no interest for the user)"
        extends Modelica.Icons.UtilitiesPackage;

        function initialStateWithXorshift64star "Return an initial state vector for a random number generator (based on xorshift64star algorithm)"
          import Modelica.Math.Random.Generators.Xorshift64star;
          extends Modelica.Icons.Function;
          input Integer localSeed "The local seed to be used for generating initial states";
          input Integer globalSeed "The global seed to be combined with the local seed";
          input Integer nState(min = 1) "The dimension of the state vector (>= 1)";
          output Integer[nState] state "The generated initial states";
        protected
          Real r "Random number only used inside function";
          Integer[2] aux "Intermediate container of state integers";
          Integer nStateEven "Highest even number <= nState";
        algorithm
          aux := Xorshift64star.initialState(localSeed, globalSeed);
          if nState >= 2 then
            state[1:2] := aux;
          else
            state[1] := aux[1];
          end if;
          nStateEven := 2 * div(nState, 2);
          for i in 3:2:nStateEven loop
            (r, aux) := Xorshift64star.random(state[i - 2:i - 1]);
            state[i:i + 1] := aux;
          end for;
          if nState >= 3 and nState <> nStateEven then
            (r, aux) := Xorshift64star.random(state[nState - 2:nState - 1]);
            state[nState] := aux[1];
          else
          end if;
        end initialStateWithXorshift64star;

        impure function automaticGlobalSeed "Creates an automatic integer seed (typically from the current time and process id; this is an impure function)"
          extends Modelica.Icons.Function;
          output Integer seed "Automatically generated seed";
          external "C" seed = ModelicaRandom_automaticGlobalSeed(0.0) annotation(IncludeDirectory = "modelica://Modelica/Resources/C-Sources", Library = "ModelicaExternalC");
        end automaticGlobalSeed;

        function automaticLocalSeed "Creates an automatic local seed from the instance name"
          extends Modelica.Icons.Function;
          input String path "Full path name of the instance (inquire with getInstanceName())";
          output Integer seed "Automatically generated seed";
        algorithm
          seed := Modelica.Utilities.Strings.hashString(path);
        end automaticLocalSeed;

        function initializeImpureRandom "Initializes the internal state of the impure random number generator"
          extends Modelica.Icons.Function;
          input Integer seed "The input seed to initialize the impure random number generator";
          output Integer id "Identification number to be passed as input to function impureRandom, in order that sorting is correct";
        protected
          constant Integer localSeed = 715827883 "Since there is no local seed, a large prime number is used";
          Integer[33] rngState "The internal state vector of the impure random number generator";

          impure function setInternalState "Stores the given state vector in an external static variable"
            extends Modelica.Icons.Function;
            input Integer[33] rngState "The initial state";
            input Integer id;
            external "C" ModelicaRandom_setInternalState_xorshift1024star(rngState, size(rngState, 1), id) annotation(IncludeDirectory = "modelica://Modelica/Resources/C-Sources", Library = "ModelicaExternalC");
          end setInternalState;
        algorithm
          rngState := initialStateWithXorshift64star(localSeed, seed, size(rngState, 1));
          id := localSeed;
          setInternalState(rngState, id);
        end initializeImpureRandom;
      end Utilities;
    end Random;

    package Distributions "Library of distribution functions"
      extends Modelica.Icons.Package;

      package Normal "Library of normal distribution functions"
        extends Modelica.Icons.Package;

        function quantile "Quantile of normal distribution"
          import Modelica.Math.Special;
          extends Modelica.Math.Distributions.Interfaces.partialQuantile;
          input Real mu = 0 "Expectation (mean) value of the normal distribution";
          input Real sigma = 1 "Standard deviation of the normal distribution";
        algorithm
          y := mu + sigma * sqrt(2) * Special.erfInv(2 * u - 1);
          annotation(Inline = true);
        end quantile;
      end Normal;

      package Interfaces "Library of interfaces for distribution functions"
        extends Modelica.Icons.InterfacesPackage;

        partial function partialQuantile "Common interface of quantile functions (= inverse cumulative distribution functions)"
          extends Modelica.Icons.Function;
          input Real u(min = 0, max = 1) "Random number in the range 0 <= u <= 1";
          output Real y "Random number u transformed according to the given distribution";
        end partialQuantile;
      end Interfaces;
    end Distributions;

    package Special "Library of special mathematical functions"
      extends Modelica.Icons.Package;

      function erfInv "Inverse error function: u = erf(erfInv(u))"
        extends Modelica.Icons.Function;
        input Real u "Input argument in the range -1 <= u <= 1";
        output Real y "= inverse of error function";
      algorithm
        if u >= 1 then
          y := Modelica.Constants.inf;
        elseif u <= (-1) then
          y := -Modelica.Constants.inf;
        elseif u == 0 then
          y := 0;
        elseif u < 0 then
          y := -Internal.erfInvUtil(-u, 1 + u);
        else
          y := Internal.erfInvUtil(u, 1 - u);
        end if;
        annotation(smoothOrder = 1);
      end erfInv;

      package Internal "Internal utility functions that should not be directly utilized by the user"
        extends Modelica.Icons.InternalPackage;

        function polyEval "Evaluate a polynomial c[1] + c[2]*u + c[3]*u^2 + ...."
          extends Modelica.Icons.Function;
          input Real[:] c "Polynomial coefficients";
          input Real u "Abscissa value";
          output Real y "= c[1] + u*(c[2] + u*(c[3] + u*(c[4]*u^3 + ...)))";
        algorithm
          y := c[size(c, 1)];
          for j in size(c, 1) - 1:(-1):1 loop
            y := c[j] + u * y;
          end for;
        end polyEval;

        function erfInvUtil "Utility function for erfInv(u) and erfcInv(u)"
          extends Modelica.Icons.Function;
          input Real p "First input argument";
          input Real q "Second input argument";
          output Real y "Result value";
        protected
          Real g;
          Real r;
          Real xs;
          Real x;
          constant Real y1 = 0.0891314744949340820313;
          constant Real[8] P1 = {-0.000508781949658280665617, -0.00836874819741736770379, 0.0334806625409744615033, -0.0126926147662974029034, -0.0365637971411762664006, 0.0219878681111168899165, 0.00822687874676915743155, -0.00538772965071242932965};
          constant Real[10] Q1 = {1.0, -0.970005043303290640362, -1.56574558234175846809, 1.56221558398423026363, 0.662328840472002992063, -0.71228902341542847553, -0.0527396382340099713954, 0.0795283687341571680018, -0.00233393759374190016776, 0.000886216390456424707504};
          constant Real y2 = 2.249481201171875;
          constant Real[9] P2 = {-0.202433508355938759655, 0.105264680699391713268, 8.37050328343119927838, 17.6447298408374015486, -18.8510648058714251895, -44.6382324441786960818, 17.445385985570866523, 21.1294655448340526258, -3.67192254707729348546};
          constant Real[9] Q2 = {1.0, 6.24264124854247537712, 3.9713437953343869095, -28.6608180499800029974, -20.1432634680485188801, 48.5609213108739935468, 10.8268667355460159008, -22.6436933413139721736, 1.72114765761200282724};
          constant Real y3 = 0.807220458984375;
          constant Real[11] P3 = {-0.131102781679951906451, -0.163794047193317060787, 0.117030156341995252019, 0.387079738972604337464, 0.337785538912035898924, 0.142869534408157156766, 0.0290157910005329060432, 0.00214558995388805277169, -0.679465575181126350155e-6, 0.285225331782217055858e-7, -0.681149956853776992068e-9};
          constant Real[8] Q3 = {1.0, 3.46625407242567245975, 5.38168345707006855425, 4.77846592945843778382, 2.59301921623620271374, 0.848854343457902036425, 0.152264338295331783612, 0.01105924229346489121};
          constant Real y4 = 0.93995571136474609375;
          constant Real[9] P4 = {-0.0350353787183177984712, -0.00222426529213447927281, 0.0185573306514231072324, 0.00950804701325919603619, 0.00187123492819559223345, 0.000157544617424960554631, 0.460469890584317994083e-5, -0.230404776911882601748e-9, 0.266339227425782031962e-11};
          constant Real[7] Q4 = {1.0, 1.3653349817554063097, 0.762059164553623404043, 0.220091105764131249824, 0.0341589143670947727934, 0.00263861676657015992959, 0.764675292302794483503e-4};
          constant Real y5 = 0.98362827301025390625;
          constant Real[9] P5 = {-0.0167431005076633737133, -0.00112951438745580278863, 0.00105628862152492910091, 0.000209386317487588078668, 0.149624783758342370182e-4, 0.449696789927706453732e-6, 0.462596163522878599135e-8, -0.281128735628831791805e-13, 0.99055709973310326855e-16};
          constant Real[7] Q5 = {1.0, 0.591429344886417493481, 0.138151865749083321638, 0.0160746087093676504695, 0.000964011807005165528527, 0.275335474764726041141e-4, 0.282243172016108031869e-6};
          constant Real y6 = 0.99714565277099609375;
          constant Real[8] P6 = {-0.0024978212791898131227, -0.779190719229053954292e-5, 0.254723037413027451751e-4, 0.162397777342510920873e-5, 0.396341011304801168516e-7, 0.411632831190944208473e-9, 0.145596286718675035587e-11, -0.116765012397184275695e-17};
          constant Real[7] Q6 = {1.0, 0.207123112214422517181, 0.0169410838120975906478, 0.000690538265622684595676, 0.145007359818232637924e-4, 0.144437756628144157666e-6, 0.509761276599778486139e-9};
          constant Real y7 = 0.99941349029541015625;
          constant Real[8] P7 = {-0.000539042911019078575891, -0.28398759004727721098e-6, 0.899465114892291446442e-6, 0.229345859265920864296e-7, 0.225561444863500149219e-9, 0.947846627503022684216e-12, 0.135880130108924861008e-14, -0.348890393399948882918e-21};
          constant Real[7] Q7 = {1.0, 0.0845746234001899436914, 0.00282092984726264681981, 0.468292921940894236786e-4, 0.399968812193862100054e-6, 0.161809290887904476097e-8, 0.231558608310259605225e-11};
        algorithm
          if p <= 0.5 then
            g := p * (p + 10);
            r := polyEval(P1, p) / polyEval(Q1, p);
            y := g * y1 + g * r;
          elseif q >= 0.25 then
            g := sqrt(-2 * log(q));
            xs := q - 0.25;
            r := polyEval(P2, xs) / polyEval(Q2, xs);
            y := g / (y2 + r);
          else
            x := sqrt(-log(q));
            if x < 3 then
              xs := x - 1.125;
              r := polyEval(P3, xs) / polyEval(Q3, xs);
              y := y3 * x + r * x;
            elseif x < 6 then
              xs := x - 3;
              r := polyEval(P4, xs) / polyEval(Q4, xs);
              y := y4 * x + r * x;
            elseif x < 18 then
              xs := x - 6;
              r := polyEval(P5, xs) / polyEval(Q5, xs);
              y := y5 * x + r * x;
            elseif x < 44 then
              xs := x - 18;
              r := polyEval(P6, xs) / polyEval(Q6, xs);
              y := y6 * x + r * x;
            else
              xs := x - 44;
              r := polyEval(P7, xs) / polyEval(Q7, xs);
              y := y7 * x + r * x;
            end if;
          end if;
        end erfInvUtil;
      end Internal;
    end Special;

    package Icons "Icons for Math"
      extends Modelica.Icons.IconsPackage;

      partial function AxisLeft "Basic icon for mathematical function with y-axis on left side" end AxisLeft;

      partial function AxisCenter "Basic icon for mathematical function with y-axis in the center" end AxisCenter;
    end Icons;

    function sin "Sine"
      extends Modelica.Math.Icons.AxisLeft;
      input Modelica.Units.SI.Angle u "Independent variable";
      output Real y "Dependent variable y=sin(u)";
      external "builtin" y = sin(u);
    end sin;

    function cos "Cosine"
      extends Modelica.Math.Icons.AxisLeft;
      input Modelica.Units.SI.Angle u "Independent variable";
      output Real y "Dependent variable y=cos(u)";
      external "builtin" y = cos(u);
    end cos;

    function asin "Inverse sine (-1 <= u <= 1)"
      extends Modelica.Math.Icons.AxisCenter;
      input Real u "Independent variable";
      output Modelica.Units.SI.Angle y "Dependent variable y=asin(u)";
      external "builtin" y = asin(u);
    end asin;

    function atan2 "Four quadrant inverse tangent"
      extends Modelica.Math.Icons.AxisCenter;
      input Real u1 "First independent variable";
      input Real u2 "Second independent variable";
      output Modelica.Units.SI.Angle y "Dependent variable y=atan2(u1, u2)=atan(u1/u2)";
      external "builtin" y = atan2(u1, u2);
    end atan2;

    function asinh "Inverse of sinh (area hyperbolic sine)"
      extends Modelica.Math.Icons.AxisCenter;
      input Real u "Independent variable";
      output Real y "Dependent variable y=asinh(u)";
    algorithm
      y := Modelica.Math.log(u + sqrt(u * u + 1));
    end asinh;

    function exp "Exponential, base e"
      extends Modelica.Math.Icons.AxisCenter;
      input Real u "Independent variable";
      output Real y "Dependent variable y=exp(u)";
      external "builtin" y = exp(u);
    end exp;

    function log "Natural (base e) logarithm (u shall be > 0)"
      extends Modelica.Math.Icons.AxisLeft;
      input Real u "Independent variable";
      output Real y "Dependent variable y=ln(u)";
      external "builtin" y = log(u);
    end log;
  end Math;

  package Utilities "Library of utility functions dedicated to scripting (operating on files, streams, strings, system)"
    extends Modelica.Icons.UtilitiesPackage;

    package Streams "Read from files and write to files"
      extends Modelica.Icons.FunctionsPackage;

      function error "Print error message and cancel all actions - in case of an unrecoverable error"
        extends Modelica.Icons.Function;
        input String string "String to be printed to error message window";
        external "C" ModelicaError(string) annotation(IncludeDirectory = "modelica://Modelica/Resources/C-Sources", Include = "#include \"ModelicaUtilities.h\"", Library = "ModelicaExternalC");
      end error;
    end Streams;

    package Strings "Operations on strings"
      extends Modelica.Icons.FunctionsPackage;

      function hashString "Create a hash value of a string"
        extends Modelica.Icons.Function;
        input String string "The string to create a hash from";
        output Integer hash "The hash value of string";
        external "C" hash = ModelicaStrings_hashString(string) annotation(IncludeDirectory = "modelica://Modelica/Resources/C-Sources", Include = "#include \"ModelicaStrings.h\"", Library = "ModelicaExternalC");
      end hashString;
    end Strings;

    package Internal "Internal components that a user should usually not directly utilize"
      extends Modelica.Icons.InternalPackage;
      import Modelica.Units.SI;

      package PartialModelicaServices "Interfaces of components requiring a tool specific implementation"
        extends Modelica.Icons.InternalPackage;

        package Animation "Models and functions for 3-dim. animation"
          extends Modelica.Icons.Package;

          partial model PartialShape "Interface for 3D animation of elementary shapes"
            import Modelica.Mechanics.MultiBody.Frames;
            import Modelica.Mechanics.MultiBody.Types;
            parameter Types.ShapeType shapeType = "box" "Type of shape (box, sphere, cylinder, pipecylinder, cone, pipe, beam, gearwheel, spring, <external shape>)";
            input Frames.Orientation R = Frames.nullRotation() "Orientation object to rotate the world frame into the object frame";
            input SI.Position[3] r = {0, 0, 0} "Position vector from origin of world frame to origin of object frame, resolved in world frame";
            input SI.Position[3] r_shape = {0, 0, 0} "Position vector from origin of object frame to shape origin, resolved in object frame";
            input Real[3] lengthDirection(each final unit = "1") = {1, 0, 0} "Vector in length direction, resolved in object frame";
            input Real[3] widthDirection(each final unit = "1") = {0, 1, 0} "Vector in width direction, resolved in object frame";
            input SI.Length length = 0 "Length of visual object";
            input SI.Length width = 0 "Width of visual object";
            input SI.Length height = 0 "Height of visual object";
            input Types.ShapeExtra extra = 0.0 "Additional size data for some of the shape types";
            input Real[3] color = {255, 0, 0} "Color of shape";
            input Types.SpecularCoefficient specularCoefficient = 0.7 "Reflection of ambient light (= 0: light is completely absorbed)";
          end PartialShape;

          partial model PartialSurface "Interface for 3D animation of surfaces"
            import Modelica.Mechanics.MultiBody.Frames;
            import Modelica.Mechanics.MultiBody.Types;
            input Frames.Orientation R = Frames.nullRotation() "Orientation object to rotate the world frame into the surface frame";
            input SI.Position[3] r_0 = {0, 0, 0} "Position vector from origin of world frame to origin of surface frame, resolved in world frame";
            parameter Integer nu = 2 "Number of points in u-Dimension";
            parameter Integer nv = 2 "Number of points in v-Dimension";
            replaceable function surfaceCharacteristic = Modelica.Mechanics.MultiBody.Interfaces.partialSurfaceCharacteristic "Function defining the surface characteristic" annotation(choicesAllMatching = true);
            parameter Boolean wireframe = false "= true: 3D model will be displayed without faces";
            parameter Boolean multiColoredSurface = false "= true: Color is defined for each surface point";
            input Real[3] color = {255, 0, 0} "Color of surface";
            input Types.SpecularCoefficient specularCoefficient = 0.7 "Reflection of ambient light (= 0: light is completely absorbed)";
            input Real transparency = 0 "Transparency of shape: 0 (= opaque) ... 1 (= fully transparent)";
          end PartialSurface;
        end Animation;
      end PartialModelicaServices;
    end Internal;
  end Utilities;

  package Constants "Library of mathematical constants and constants of nature (e.g., pi, eps, R, sigma)"
    extends Modelica.Icons.Package;
    import Modelica.Units.SI;
    import Modelica.Units.NonSI;
    final constant Real pi = 2 * Modelica.Math.asin(1.0);
    final constant Real eps = ModelicaServices.Machine.eps "Biggest number such that 1.0 + eps = 1.0";
    final constant Real inf = ModelicaServices.Machine.inf "Biggest Real number such that inf and -inf are representable on the machine";
    final constant SI.Velocity c = 299792458 "Speed of light in vacuum";
    final constant SI.Acceleration g_n = 9.80665 "Standard acceleration of gravity on earth";
    final constant SI.ElectricCharge q = 1.602176634e-19 "Elementary charge";
    final constant Real h(final unit = "J.s") = 6.62607015e-34 "Planck constant";
    final constant Real k(final unit = "J/K") = 1.380649e-23 "Boltzmann constant";
    final constant Real N_A(final unit = "1/mol") = 6.02214076e23 "Avogadro constant";
    final constant Real mu_0(final unit = "N/A2") = 4 * pi * 1.00000000055e-7 "Magnetic constant";
  end Constants;

  package Icons "Library of icons"
    extends Icons.Package;

    partial package Package "Icon for standard packages" end Package;

    partial package InterfacesPackage "Icon for packages containing interfaces"
      extends Modelica.Icons.Package;
    end InterfacesPackage;

    partial package SourcesPackage "Icon for packages containing sources"
      extends Modelica.Icons.Package;
    end SourcesPackage;

    partial package UtilitiesPackage "Icon for utility packages"
      extends Modelica.Icons.Package;
    end UtilitiesPackage;

    partial package TypesPackage "Icon for packages containing type definitions"
      extends Modelica.Icons.Package;
    end TypesPackage;

    partial package FunctionsPackage "Icon for packages containing functions"
      extends Modelica.Icons.Package;
    end FunctionsPackage;

    partial package IconsPackage "Icon for packages containing icons"
      extends Modelica.Icons.Package;
    end IconsPackage;

    partial package InternalPackage "Icon for an internal package (indicating that the package should not be directly utilized by user)" end InternalPackage;

    partial function Function "Icon for functions" end Function;

    partial record Record "Icon for records" end Record;

    type TypeReal "Icon for Real types"
      extends Real;
    end TypeReal;

    type TypeInteger "Icon for Integer types"
      extends Integer;
    end TypeInteger;

    type TypeString "Icon for String types"
      extends String;
    end TypeString;
  end Icons;

  package Units "Library of type and unit definitions"
    extends Modelica.Icons.Package;

    package SI "Library of SI unit definitions"
      extends Modelica.Icons.Package;
      type Angle = Real(final quantity = "Angle", final unit = "rad", displayUnit = "deg");
      type Length = Real(final quantity = "Length", final unit = "m");
      type Position = Length;
      type Distance = Length(min = 0);
      type Diameter = Length(min = 0);
      type Time = Real(final quantity = "Time", final unit = "s");
      type AngularVelocity = Real(final quantity = "AngularVelocity", final unit = "rad/s");
      type Velocity = Real(final quantity = "Velocity", final unit = "m/s");
      type Acceleration = Real(final quantity = "Acceleration", final unit = "m/s2");
      type Period = Real(final quantity = "Time", final unit = "s");
      type Frequency = Real(final quantity = "Frequency", final unit = "Hz");
      type AngularFrequency = Real(final quantity = "AngularFrequency", final unit = "rad/s");
      type Force = Real(final quantity = "Force", final unit = "N");
      type Torque = Real(final quantity = "Torque", final unit = "N.m");
      type ElectricCharge = Real(final quantity = "ElectricCharge", final unit = "C");
      type FaradayConstant = Real(final quantity = "FaradayConstant", final unit = "C/mol");
    end SI;

    package NonSI "Type definitions of non SI and other units"
      extends Modelica.Icons.Package;
      type Temperature_degC = Real(final quantity = "ThermodynamicTemperature", final unit = "degC") "Absolute temperature in degree Celsius (for relative temperature use Modelica.Units.SI.TemperatureDifference)" annotation(absoluteValue = true);
    end NonSI;

    package Conversions "Conversion functions to/from non SI units and type definitions of non SI units"
      extends Modelica.Icons.Package;

      function to_unit1 "Change the unit of a Real number to unit=\"1\""
        extends Modelica.Units.Icons.Conversion;
        input Real r "Real number";
        output Real result(unit = "1") "Real number r with unit=\"1\"";
      algorithm
        result := r;
        annotation(Inline = true);
      end to_unit1;
    end Conversions;

    package Icons "Icons for Units"
      extends Modelica.Icons.IconsPackage;

      partial function Conversion "Base icon for conversion functions" end Conversion;
    end Icons;
  end Units;
  annotation(version = "4.0.0", versionDate = "2020-06-04", dateModified = "2020-06-04 11:00:00Z");
end Modelica;

package SuspensionSystem
  import SI = Modelica.Units.SI;

  package Components
    model RoadProfile
      parameter Real roadRoughness = 3 "Road height StdDeviation in cm";
      Modelica.Mechanics.MultiBody.Joints.Prismatic Road(animation = true, boxColor = {140, 140, 140}, boxHeight = 1, boxWidth = 0.3, n = {0, 1, 0}, s(start = 0.5), useAxisFlange = true);
      Modelica.Mechanics.Translational.Sources.Position position(a(fixed = false), exact = false, v(fixed = false));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b zR;
      Modelica.Blocks.Continuous.Filter LPF(f_cut = 1, filterType = Modelica.Blocks.Types.FilterType.LowPass, gain = roadRoughness, order = 2);
      Modelica.Blocks.Noise.NormalNoise normalNoise(enableNoise = true, mu = 0, samplePeriod = 0.01, sigma = 0.05, startTime = 0.1, useAutomaticLocalSeed = false, useGlobalSeed = false);
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a world_a;
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation y(r = {0, -0.5, 0});
      Modelica.Blocks.Interfaces.RealInput RoadRoughness;
      Modelica.Blocks.Math.Product product;
      Modelica.Blocks.Math.Add add;
      Modelica.Blocks.Sources.RealExpression offset(y = 0.5);
    equation
      connect(position.support, Road.support);
      connect(position.flange, Road.axis);
      connect(zR, Road.frame_b);
      connect(normalNoise.y, LPF.u);
      connect(world_a, y.frame_a);
      connect(y.frame_b, Road.frame_a);
      connect(LPF.y, product.u1);
      connect(RoadRoughness, product.u2);
      connect(add.y, position.s_ref);
      connect(product.y, add.u1);
      connect(offset.y, add.u2);
    end RoadProfile;
  end Components;
end SuspensionSystem;

model RoadProfile_total
  extends SuspensionSystem.Components.RoadProfile;
end RoadProfile_total;
