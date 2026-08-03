# Demo videos

## Play on GitHub

Repository `.html` pages are **not** executed by github.com’s markdown
viewer (clicking them shows source). To play **inside GitHub**:

1. Click the poster in the README / phase report — it links to the `.mp4`
   on the `blob/main/...` page, where GitHub’s built-in video viewer runs.
2. Phase 7.3 also has an inline README stream via GitHub
   `user-attachments` (bare URL in the README).

| Demo | GitHub video viewer (click) | Offline HTML | mp4 |
|------|----------------------------|--------------|-----|
| Phase 7.3 densest 2×20 | [blob viewer](https://github.com/jywilson2/spark_isaac_mycobot_v3/blob/main/docs/videos/mycobot_280_m5_2x20.mp4) | [`mycobot_280_m5_2x20.html`](mycobot_280_m5_2x20.html) | [`…mp4`](mycobot_280_m5_2x20.mp4) |
| Phase 7.5 `n6-4-8` seed4242 | [blob viewer](https://github.com/jywilson2/spark_isaac_mycobot_v3/blob/main/docs/videos/phase7_5-variable_dz0_30_n6-4-8_seed4242.mp4) | [`phase7_5-…html`](phase7_5-variable_dz0_30_n6-4-8_seed4242.html) | [`…mp4`](phase7_5-variable_dz0_30_n6-4-8_seed4242.mp4) |

Phase 7.3 inline attachment (README-only stream):
`https://github.com/user-attachments/assets/e1632486-8215-4b7e-8963-d726cd621b28`

## Play offline (local clone)

Open an `.html` file in a browser and click the poster to start the native
`<video>` element. Relative `src` paths load the sibling `.mp4` / poster.
