# Lamello Zeta P2 Pins for Fusion

An Autodesk Fusion add-in that sketches (and optionally cuts) the CNC reference holes that position a **Lamello Zeta P2** for **Clamex P / P-System** grooves. You select your panels, and it finds every joint between them and adds the holes. You don't need to draw the reference-hole sketches by hand.

> **Status: early release.** The joint-finding and hole-layout code has unit tests. The Fusion side has been checked against Autodesk's API docs but not yet used in production. Try it on a test design and do a test cut on scrap first (see [Limits and checks](#limits-and-checks)).

## The method this automates

Without a 5-axis machine or an angle head, the P-System groove can't be cut on a CNC. Lamello's workaround: the CNC drills small reference holes, and the Zeta P2 registers on them to cut the groove by hand.

| Joint side | What the CNC drills | What goes in it | Fusion user parameters |
|---|---|---|---|
| **End**: the panel whose end/edge gets the groove | One Ø6 mm hole per connector, drilled into the broad face 7.5 mm in from the end, half the panel thickness deep. This is also the Clamex P access hole. | Zeta P2 positioning clip **251067** | `ZetaEnd_Pin`, `ZetaEnd_Inset`, `ZetaEnd_Depth` |
| **Face**: the panel whose face receives the other panel's end | Two Ø5 mm holes per connector, 101 mm apart, centred on the connector and on the mating panel's mid-thickness line, 6 mm+ deep | Zeta P2 positioning pins **251048** (Ø5) or **251066** (Ø8) | `ZetaFace_Pin`, `ZetaFace_PinSpace`, `ZetaFace_Depth` |

The face pin holes sit on the groove centreline, so once the joint is assembled they're hidden under the mating panel's end.

## Install

1. Download this repository: **Code → Download ZIP**, then unzip it. Or `git clone` it.
2. In Fusion, open **Utilities → Add-Ins → Scripts and Add-Ins** (Shift+S).
3. On the **Add-Ins** tab, click **+** (Add) and choose the **`LamelloZetaPins`** folder inside the download.

   You can also copy that folder into Fusion's add-ins folder:
   - macOS: `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/`
   - Windows: `%appdata%\Autodesk\Autodesk Fusion 360\API\AddIns\`
4. Select **LamelloZetaPins**, click **Run**, and tick **Run on Startup**.
5. A **LAMELLO** panel with the **Zeta P2 Pins** button appears on the Solid tab. If the panel can't be created, the button goes under **Create** instead.

## Use

### Joint mode (normal use)

1. Click **Zeta P2 Pins** and select **two or more panels** (bodies) that touch. A whole carcass at once is fine.
2. Every touching pair is analysed:
   - **End-to-face** (side under a top, top between sides, shelf into a side): the end panel gets Ø6 end holes and the other panel gets Ø5 pin pairs.
   - **Edge-to-edge** (panel glue-ups): both panels get Ø6 end holes.
   - **Face-to-face** (laminated) contacts are skipped and listed in the summary.
3. Set the layout:
   - **Edge distance**: from each end of the joint to the first and last connector.
   - **Max spacing**: the add-in picks the fewest connectors that keep the gaps under this. Or switch to **Fixed count**.
4. Check the preview and the summary box, then click **OK**.

**Which face the end holes go into:**
- Corner joints: the face toward the inside of the carcass.
- T-joints and edge-to-edge joints: the underside (−Z, falling back to −Y then −X).

**Flip end-hole side** swaps this for every joint in the run. To flip only one joint, run the command on just those two panels.

### Single panel mode

Use this when the mating panel isn't modelled:
- **End**: pick the broad face and the end edge. You get Ø6 holes at `ZetaEnd_Inset` from the edge.
- **Face**: pick the face and any straight edge parallel to the groove, and set **Groove offset** = distance from that edge to the groove centreline (for example, shelf position + half the shelf thickness).

### Options

- **Cut holes**: adds an extrude-cut (flat-bottom) per sketch, limited to that panel's body. Untick it for sketches only.
- **End hole depth**: *Half panel thickness*, measured per panel, or your `ZetaEnd_Depth` parameter.

### What you get

- One sketch per panel per joint, named like `Zeta End - Left Side:1 (to Top:1)` / `Zeta Face - Top:1 (from Left Side:1)`.
- Circle centres are fixed, all circles are set equal, and one diameter dimension is driven by `ZetaEnd_Pin` / `ZetaFace_Pin`.
- Pin pairs are joined by a construction line.
- The hole depth is driven by `ZetaFace_Depth`, or by `ZetaEnd_Depth` when you choose it.
- Each run is grouped in the timeline as **Lamello Zeta Pins**.
- Missing user parameters are created with these defaults: 6 / 7.5 / 5 / 101 / 6 mm. Existing ones are never changed, so set your own values in **Modify → Change Parameters**.

For CAM, use a Drill operation with **Select same diameter**, or select the sketch centre points.

## Limits and checks

- **Check the first run.** On a test design, use **Inspect → Measure** to confirm a few holes:
  - end hole 7.5 mm from the end, half-thickness deep, on the expected face;
  - pins 101 mm apart, centred on the mating panel's thickness.

  Also try a panel nested two components deep. If the add-in can't map assembly coordinates correctly, it stops with an error rather than drilling in the wrong place.
- **Do a test cut on scrap before production.** The add-in assumes:
  - the groove centre is midway between the two pin holes, on their line;
  - the groove centre lines up with the clip hole along the edge.

  The 7.5 mm inset and half-thickness depth are working shop values, not taken from a Lamello drawing. If your results differ, change the user parameters.
- Handles butt joints between flat, rectangular-ish panels. Mitred and angled joints are reported and skipped.
- Hole **positions** are computed values, not parametric. If you change `ZetaEnd_Inset` or `ZetaFace_PinSpace` later, delete the old sketches/cuts and re-run. Diameters and depths do follow the parameters.
- If a panel is a component used more than once, holes added to it appear on **every copy**. The summary warns you when that happens.
- Linked (external) components can't be edited. Open them, or break the link first.
- Warnings tell you when:
  - a pin would show past the end of the mating panel: increase the edge distance;
  - a hole falls off a panel: it's left as a construction circle and not cut.

## Development

- [`LamelloZetaPins/LamelloZetaPins.py`](LamelloZetaPins/LamelloZetaPins.py): Fusion command, UI, and sketch/cut creation.
- [`LamelloZetaPins/zeta_geometry.py`](LamelloZetaPins/zeta_geometry.py): pure-Python joint detection and hole layout (no Fusion imports).
- [`tests/test_zeta_geometry.py`](tests/test_zeta_geometry.py): geometry tests. Run them from the repository root:

  ```bash
  python3 -m unittest discover -s tests -v
  ```

After editing the add-in, stop and re-run it in Fusion. If a change doesn't show up, restart Fusion, because Python caches `zeta_geometry`.

Issues and pull requests are welcome. Reports from real cuts (photos, measured offsets) are especially useful.

## Sources

- [Positioning pins D5 mm for combination with CNC (Gamma Fittings)](https://gammafittings.co.uk/product/positioning-pins-d5-mm-for-combination-with-cnc): two Ø5/Ø8 holes, 101 mm centres
- [Lamello P-System CNC (CSAW)](https://www.csaw.com/lamello/p-system-cnc/): clip 251067 for the 6 mm Clamex access hole; pins 251048/251066
- [CNC positioning clip for Lamello Zeta P2 (Ostermann)](https://www.ostermann.eu/en_DK/product/cnc-positioning-clip-for-lamello-zeta-p2)
- [Lamello P-System drill jig 125344 (Lincoln Sentry PDF)](https://media.lincolnsentry.com.au/h6b/h24/9585522769950.pdf): Ø6 mm access-hole drill, inserts for P-10 / P-14 / P-15

## License and trademarks

Released under the [MIT License](LICENSE).

This is an independent project, not affiliated with or endorsed by Lamello AG or Autodesk. Lamello, Zeta, Clamex and P-System are trademarks of Lamello AG. Autodesk and Fusion are trademarks of Autodesk, Inc.
