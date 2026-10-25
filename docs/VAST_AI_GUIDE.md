# Running the simulation on Vast.ai — step by step

[Vast.ai](https://vast.ai) rents computers by the hour. You start one with our Docker image (everything is already
installed), run experiments for as long as you need, download the results and **destroy** the machine. A suitable
machine costs about **$0.05–0.20 per hour**; a typical session of 1–2 hours costs less than $0.50.

You need: a Vast.ai account with a little credit, and a laptop with a web browser. Part A uses only the browser.
Part B (SSH) is optional but more comfortable for long runs.

> **The one rule that matters:** a rented machine costs money every hour until you **destroy** it — also when you
> are not using it and also when it is "stopped" (stopped machines still pay for disk). Always finish with
> step 8.

---

## Part A — first session (browser only)

### 1. Create an account and add credit
1. Go to <https://cloud.vast.ai>, click **Sign up**, confirm your e-mail.
2. Open **Billing** in the left menu and add credit (the minimum, about $5–10, is enough for many sessions).
   Do **not** enable automatic top-up.

### 2. Create a template (once)
A template remembers which Docker image to start, so you do not have to type it each time.

1. Left menu → **Templates** → **+ New** (or **Create Template**).
2. Fill in:

   | Field | Value |
   |---|---|
   | Template name | `wind-corridors` |
   | Image path / tag | `abdulmannan617/wind-aware-delivery:v2` |
   | Launch mode | **Jupyter-python notebook + SSH** (tick **Use Jupyter Lab** and **Use direct HTTPS/SSH** if offered) |
   | Disk space | **40 GB** |
   | Docker options / on-start script | leave empty |

3. Save. The template now appears under **My Templates**.

### 3. Choose and rent a machine
1. Left menu → **Search**. At the top, select your template `wind-corridors`.
2. Set the filters (sliders on the left):
   - **GPU**: 1 GPU, at least **8 GB** of GPU memory (only AirSim uses the GPU; any GTX 1070 or newer is fine)
   - **CPU cores**: at least **16** (the experiments use the CPU cores — more cores = faster)
   - **Reliability** above 98 %, **Internet download** above 300 Mbps (the image is ~4 GB)
   - **Disk space** at least 40 GB
3. Sort by **Price (low to high)** and pick a machine around $0.05–0.20/h. Note its price.
4. Click **RENT**.

### 4. Wait for it to start
Left menu → **Instances**. The status goes *Loading* → *Running*. The first start downloads the image and takes
**5–15 minutes**. If it is still loading after 20 minutes, destroy it (step 8) and rent another one.

### 5. Open a terminal
1. On the running instance click **Open** (or **Jupyter**). Jupyter Lab opens in a new browser tab.
   If the browser warns about the certificate, click *Advanced → Proceed*.
2. Open a terminal: in the classic Jupyter page click **New** (top right) → **Terminal**; in Jupyter Lab use
   **File → New → Terminal**. You now have a command line on the rented machine. (Jupyter's file list starts
   in the repository folder `wind-aware-delivery`.)
3. Type (copy–paste works):

   ```bash
   cd ~/wind-aware-delivery
   git pull                    # get the newest version of the code
   cd sim
   python3 run_experiments.py --config ../configs/quick.yaml
   ```

   The first line says how many missions run in parallel (`12 missions on 16 workers`): one per CPU core you
   rented. Then progress lines follow, e.g. `12/12  elapsed 150 s  remaining ~0 s`. The quick test takes 3–6
   minutes. (Do not trust `nproc` on Vast.ai: it shows all cores of the host computer, not the ones you rented.)

4. Make the summary and figures:

   ```bash
   python3 summarise.py ../results/experiments/quick/campaign.csv
   ```

### 6. Look at the results
In Jupyter's file list go to `results/experiments/quick/` and click `success_rate.png`, `energy_per_parcel.png`
or `summary.csv`.

### 7. Download the results to your laptop
Easiest: put the whole experiment folder in one zip file, then download it.

```bash
cd ~/wind-aware-delivery
zip -r my_results.zip results/experiments results/single
```

`my_results.zip` now appears at the top of Jupyter's file list. Tick its box and click **Download** (classic
page), or right-click it → **Download** (Jupyter Lab).

### 8. DESTROY the machine
Back on the Vast.ai **Instances** page click the **trash-can icon** (Destroy) of your instance and confirm.
Check that the instance has disappeared from the list and that **Billing** shows no running charge.

Everything on the machine is deleted — download first (step 7).

---

## Part B — the same with SSH (optional, better for long runs)

### B1. Make an SSH key (once)
On your laptop open **PowerShell** (Windows) or **Terminal** (macOS/Linux):

```bash
ssh-keygen -t ed25519            # press Enter three times
```

Show the public key and copy the whole line:

```bash
# Windows PowerShell
type $env:USERPROFILE\.ssh\id_ed25519.pub
# macOS / Linux
cat ~/.ssh/id_ed25519.pub
```

On Vast.ai: **Keys** (or **Account → SSH Keys**) → **+ New / Add SSH key** → paste → save.
Add the key **before** renting the machine; keys are copied into new instances only.

### B2. Connect
On the **Instances** page click the **>_** (Connect) button. It shows a command such as

```bash
ssh -p 41234 root@123.45.67.89 -L 8080:localhost:8080
```

Paste it into PowerShell/Terminal and answer `yes` the first time.

### B3. Keep long runs alive with tmux
If your laptop sleeps or the Wi-Fi drops, a normal SSH session stops the experiment. Run long experiments
inside `tmux`:

```bash
tmux new -s exp                         # start a session called exp
cd ~/wind-aware-delivery/sim
python3 run_experiments.py --config ../configs/procedural_full.yaml
# press Ctrl+B, then D, to detach: the run continues on the server; you may close the laptop
```

Later: connect again (B2) and type `tmux attach -t exp` to see the progress.

### B4. Copy results to your laptop
From PowerShell/Terminal **on your laptop** (not in the SSH session), with the port and address of step B2:

```bash
scp -P 41234 -r root@123.45.67.89:~/wind-aware-delivery/results/experiments ./my_results
```

---

## Part C — command-line client (optional, for experienced users)

```bash
pip install vastai
vastai set api-key <your key from https://cloud.vast.ai/manage-keys/>
vastai search offers 'gpu_ram>=8 cpu_cores_effective>=16 reliability>0.98 inet_down>300 disk_space>=40 verified=true' -o 'dph+'
vastai create instance <ID> --image abdulmannan617/wind-aware-delivery:v2 --disk 40 --ssh --direct
vastai show instances                   # status and ID of your instance
vastai ssh-url <INSTANCE_ID>            # address to connect to
vastai destroy instance <INSTANCE_ID>   # ALWAYS at the end (answer y)
```

---

## Choosing a machine for the job

| Job | Missions | Suggested machine | Time |
|---|---|---|---|
| `quick.yaml` | 12 | any, 8+ cores | 3–6 min |
| `baseline_small.yaml` and similar | 48 | 16+ cores | 5–10 min |
| `strong_gusts.yaml` | 288 | 32 cores | 10–15 min |
| `procedural_full.yaml` | 1,536 | 32 cores | 45–60 min |
| AirSim replay or demo | 1–4 flights | GPU with 8+ GB | 15–60 min |

A mission takes about 50 s on one core; the run time is roughly *missions × 50 s ÷ cores*.
Use `--dry-run` to count the missions of a config before you start it.

## Troubleshooting

| Problem | What to do |
|---|---|
| Instance stays *Loading* for more than 20 min | Destroy it, rent a machine with faster internet download |
| `Permission denied (publickey)` with SSH | The key was added after renting: use Jupyter (Part A) or rent again |
| `git pull` says *Your local changes would be overwritten* | You edited a file of the repository. Save your own configs under new names, then `git stash && git pull` |
| `No space left on device` | Rent with more disk, or delete old `results/experiments/*` folders |
| The run stopped when the laptop slept | Use `tmux` (B3) or keep the Jupyter tab open |
| `start_airsim` says *AirSim stopped* | See [AIRSIM.md](AIRSIM.md), troubleshooting |
| You forgot what is running | `htop` shows the busy processes (press `q` to quit) |
