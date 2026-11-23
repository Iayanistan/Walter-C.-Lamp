"""ASTM D4607 (iodine number of activated carbon) on the FAIRINO FR3, start to finish.

    python -m walter_fr3.d4607                       # dry run: no robot, virtual clock
    python -m walter_fr3.d4607 --ask                 # dry run that stops at every operator step
    python -m walter_fr3.d4607 --ip 192.168.58.2     # the real FR3

Who does what (section numbers are the standard's):

    8, 9, 10.1, 10.2   solutions, standardization, grinding, drying      operator (checklist + N1, N2 recorded)
    11.4               carbon dosage estimate                            Python (calc.dosages)
    10.3               weigh three carbon samples into flasks            operator (masses recorded)
    10.4               pipet 10.0 mL 5% HCl, stopper                     operator
    10.4               swirl until wetted                                ARM   swirl()
    10.4               loosen stoppers                                   operator
    10.4               onto the hot plate, boil 30 +/- 2 s, off, cool    ARM   pick() place() boil() place() cool()
    10.5               100.0 mL iodine, staggered per flask              ARM holds, dispense()
    10.5               stopper                                           operator
    10.5               shake 30 +/- 1 s                                  ARM   shake()
    10.5               unstopper, filter through folded paper            operator pulls the stopper, ARM pour()
    10.6               discard first 20 to 30 mL, collect the rest       operator
    10.6               mix the filtrate by swirling the beaker           ARM   swirl()
    10.6               pipet 50.0 mL, titrate with thiosulfate           operator (volume recorded)
    11, 12             X/M, C, log-log fit, iodine number, report        Python (calc.py)

One gripper cannot weigh, pipet, work a ground-glass stopper on a flask it is holding, or read a
titration endpoint, so those stay with the operator. Everything else is the arm.
"""

from __future__ import annotations

from . import calc
from .helpers import Walter
from .session import open_walter

FLASKS = [   # flask, rack slot, cooling spot, funnel
    ("flask_1", "rack_1", "cool_pad", "funnel"),
    ("flask_2", "rack_2", "cool_pad_2", "funnel_2"),
    ("flask_3", "rack_3", "cool_pad_3", "funnel_3"),
]
HCL_ML, IODINE_ML, FILTRATE_ML = 10.0, 100.0, 50.0

def preparation(w: Walter) -> tuple[float, float]:
    """Sections 8, 9, 10.1, 10.2 and 11.4: nothing here is arm work."""
    w.step_spec = "8-9"
    w.operator("Solutions ready: 5% HCl, 0.100 N thiosulfate (aged 4+ days), 0.100 N iodine, fresh starch")
    n1 = w.record("N1", "Thiosulfate normality from the 9.1 standardization (triplicate average)", 0.1)
    n2 = w.record("N2", "Iodine normality from the 9.2 standardization, done just before use", 0.1)
    if not 0.099 <= n2 <= 0.101:
        w.operator(f"Iodine is {n2:.4f} N, outside 0.100 +/- 0.001 N. The standard says remake it (8.3, 9.2). "
                   "Confirm to continue anyway, or abort")
    w.step_spec = "10.1-10.2"
    w.operator("Carbon ground (95% through 100 mesh), dried per D2867 and cooled in a desiccator")
    w.step_spec = "11.4"
    e = w.record("E", "Estimated iodine number of this carbon, mg/g (for the dosage estimate)", 900)
    masses = calc.dosages(n2, e, i_ml=IODINE_ML, h_ml=HCL_ML, f_ml=FILTRATE_ML)
    print("\n  Suggested carbon masses (11.4): " + ", ".join(f"{m:.3f} g" for m in masses))
    w.step_spec = None
    return n1, n2


def setup(w: Walter) -> dict[str, float]:
    """10.3 and the start of 10.4: weigh, acidify, stopper, load the rack."""
    masses = {}
    w.step_spec = "10.3"
    for flask, rack, _, _ in FLASKS:
        masses[flask] = w.record(f"mass_{flask}", f"Mass of dry carbon weighed into {flask}, g (nearest mg)")
    w.step_spec = "10.4"
    w.operator(f"Pipet {HCL_ML:.1f} mL of 5% HCl into each flask, stopper them, and set them in rack slots 1 to 3")
    for flask, rack, _, _ in FLASKS:
        w.items[rack] = flask
    w.operator("Hot plate on in the fume hood; three funnels with folded filter paper over beakers; "
               "iodine dispenser primed; arm area clear")
    w.step_spec = None
    for station in ("hotplate", "cool_pad", "cool_pad_2", "cool_pad_3", "funnel", "funnel_2", "funnel_3"):
        w.verify(station)
    w.home()
    return masses


def wet_and_boil(w: Walter):
    """10.4: swirl until wetted, then boil each flask 30 +/- 2 s and set it to cool."""
    w.step_spec = "10.4"
    for flask, rack, _, _ in FLASKS:
        w.pick(rack)
        w.swirl(seconds=10)                 # "swirl gently until the carbon is completely wetted"
        w.place(rack, verify=False)
    w.home()
    w.operator("Check the carbon in all three flasks is fully wetted, then loosen the stoppers to vent")
    for flask, rack, cool_spot, _ in FLASKS:
        w.pick(rack)
        w.place("hotplate")
        w.boil(seconds=30, tolerance_s=2)    # waits at the flask, times the boil, lifts it off on time
        w.place(cool_spot)
    w.home()
    w.cool(minutes=15)                       # "cool to room temperature"
    w.step_spec = None


def iodine_shake_filter(w: Walter):
    """10.5: one flask at a time (that is the stagger), iodine -> stopper -> shake -> filter with no waiting."""
    w.step_spec = "10.5"
    for flask, _, cool_spot, funnel in FLASKS:
        w.pick(cool_spot)
        w.dispense(ml=IODINE_ML)
        w.operator(f"Stopper {flask} firmly, right now (the arm is holding it under the dispenser)")
        w.shake(seconds=30, tolerance_s=1)
        w.move_to(funnel, "above", 1.5)      # "quickly filter": straight to the funnel
        w.operator(f"Remove the stopper from {flask}")
        w.pour(funnel, hold_s=8)
        w.place(cool_spot, verify=False)     # empty flask back out of the way
    w.home()
    w.step_spec = None


def filtrate_and_titrate(w: Walter) -> dict[str, float]:
    """10.6: the arm mixes each filtrate; pipetting and the titration endpoint are the operator's."""
    volumes = {}
    w.step_spec = "10.6"
    w.operator("Wait for all three filtrations. For each, use the first 20 to 30 mL to rinse a pipet and "
               "discard it, and collect the rest in a clean beaker")
    for flask, _, _, _ in FLASKS:
        w.operator(f"Set the clean beaker with the {flask} filtrate on the filtrate spot")
        w.items["filtrate"] = f"filtrate of {flask}"
        w.pick("filtrate")
        w.swirl(seconds=6, amplitude_deg=4, frequency_hz=1.0)   # open beaker: slower and smaller than a flask
        w.place("filtrate", verify=False)
        w.home()
        w.operator(f"Pipet {FILTRATE_ML:.1f} mL of the {flask} filtrate into a clean 250 mL flask and take the "
                   "beaker away. Titrate with thiosulfate to pale yellow, add 2 mL starch, continue to colorless")
        w.items.pop("filtrate", None)
        volumes[flask] = w.record(f"thio_{flask}", f"Thiosulfate used for {flask}, mL")
    w.step_spec = None
    return volumes


def d4607(w: Walter) -> calc.Isotherm:
    preparation(w)
    setup(w)
    wet_and_boil(w)
    iodine_shake_filter(w)
    filtrate_and_titrate(w)
    w.step_spec = "11-12"
    return w.report(hcl_ml=HCL_ML, iodine_ml=IODINE_ML, filtrate_ml=FILTRATE_ML)   # sections 11 and 12


def main(argv=None):
    s = open_walter(argv, "ASTM D4607 on the FAIRINO FR3")
    raise SystemExit(s.run(d4607))


if __name__ == "__main__":
    main()
