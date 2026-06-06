import subprocess
import ollama

from datetime import datetime, timedelta, timezone

# ============================================================
# CONFIG
# ============================================================

MODEL = "qwen2.5-coder:latest"

START_DATE = "2026-03-09"
END_DATE = "2026-05-22"

OUTPUT_FILE = "BEP_Dossier.md"

OLLAMA_OPTIONS = {
    "temperature": 0.15,
    "num_ctx": 32768,
    "num_predict": 4096,
}

# ============================================================
# DATE HELPERS
# ============================================================

def daterange_weeks(start_date, end_date):
    """
    Generate non-overlapping week ranges.
    """

    current = start_date

    while current <= end_date:

        week_end = min(
            current + timedelta(days=6),
            end_date
        )

        yield current, week_end

        current = week_end + timedelta(days=1)


# ============================================================
# GIT EXTRACTION
# ============================================================

def run_git_log(start, end):
    """
    Extract optimized git history for summarization.
    """

    cmd = [
        "git",
        "log",
        f"--since={start.strftime('%Y-%m-%d 00:00:00')}",
        f"--until={end.strftime('%Y-%m-%d 23:59:59')}",
        "--stat",
        "--summary",
        "--date=short",
        "--pretty=format:COMMIT:%n"
        "Hash: %H%n"
        "Author: %an%n"
        "Date: %ad%n"
        "Subject: %s%n"
        "Body:%n%b%n",
    ]

    try:
        output = subprocess.check_output(
            cmd
        ).decode(errors="ignore")

        if not output.strip():
            return None

        return output

    except subprocess.CalledProcessError as e:
        print(f"Git log failed: {e}")
        return None


# ============================================================
# WEEKLY DOSSIER
# ============================================================

def summarize_week(start, end, git_data):
    """
    Generate BEP-style weekly engineering dossier.
    """

    prompt = f"""
You are generating a BEP engineering dossier for a Dutch technical university bachelor project.

The dossier must align with these BEP assessment criteria:

- Technical depth
- Engineering reasoning
- Research quality
- Design justification
- Professional communication
- Systems thinking
- Collaboration indicators
- Architectural justification

You are analyzing git history as evidence of engineering work.

Your job:
- infer engineering intent
- infer architecture decisions
- infer research activities
- infer implementation strategy

DO NOT mention git commits directly.
DO NOT generate release notes.
WRITE as if this is an academic engineering dossier.

STRICT MARKDOWN OUTPUT.

### Week of {start.date()} to {end.date()}

#### Executive Summary

Provide:
- engineering progress
- systems evolved
- technical maturity
- key outcomes

#### Research & Engineering Activities

Describe:
- investigations
- experiments
- simulations
- calculations
- prototypes
- analytical work
- technical exploration

#### System & Architecture Development

Describe:
- components affected
- subsystem evolution
- interfaces
- infrastructure
- integration work

#### Technical Implementation

Describe:
- robotics algorithms
- optimization
- engineering techniques
- software systems
- validation approaches
- alternatives considered
- constraints
- scalability considerations
- maintainability implications
- engineering rationale

#### Collaboration & Project Process Indicators

Infer:
- coordination complexity
- subsystem ownership (who worked on what, use github usernames as proxies)
- multidisciplinary work
- iterative development
- workflow maturity

Git history:
{git_data}
"""

    response = ollama.chat(
        model=MODEL,
        messages=[
            {
                "role": "user",
                "content": prompt,
            }
        ],
        options=OLLAMA_OPTIONS,
    )

    return response["message"]["content"]


# ============================================================
# FINAL DOSSIER SYNTHESIS
# ============================================================

def generate_final_summary(all_weekly_reports):
    """
    Generate final BEP-style engineering synthesis.
    """

    combined = "\n\n".join(all_weekly_reports)

    prompt = f"""
You are generating the FINAL BEP engineering dossier synthesis.

The document must resemble:
- an engineering research dossier
- a technical project reflection
- an academic engineering summary

DO NOT mention commits or git history.

STRICT MARKDOWN OUTPUT.

# Overall Project Summary

Summarize:
- overall engineering trajectory
- major systems developed
- research progression
- technical complexity
- engineering maturity

## Engineering Objectives Achieved

Discuss:
- design goals achieved
- technical objectives
- analytical objectives
- implementation goals
- experimentation goals

## Research & Technical Contributions

Discuss:
- novel approaches
- engineering insight
- simulations
- modeling
- analysis
- optimization
- experimentation

## System Architecture Evolution

Describe:
- architecture progression
- subsystem interactions
- infrastructure maturity
- integration evolution
- modularity
- maintainability

## Weekly reports:
{combined}
"""

    response = ollama.chat(
        model=MODEL,
        messages=[
            {
                "role": "user",
                "content": prompt,
            }
        ],
        options=OLLAMA_OPTIONS,
    )

    return response["message"]["content"]


# ============================================================
# MAIN
# ============================================================

def main():

    start_date = datetime.fromisoformat(START_DATE)
    end_date = datetime.fromisoformat(END_DATE)

    weekly_reports = []

    print("\nGenerating BEP engineering dossier...\n")

    for week_start, week_end in daterange_weeks(
        start_date,
        end_date,
    ):

        print(
            f"Processing {week_start.date()} -> {week_end.date()}..."
        )

        git_data = run_git_log(
            week_start,
            week_end,
        )

        if not git_data:
            print("  No commits found.\n")
            continue

        report = summarize_week(
            week_start,
            week_end,
            git_data,
        )

        weekly_reports.append(report)

        print("  Weekly dossier complete.\n")

    if not weekly_reports:
        print("No reports generated.")
        return

    print("Generating final synthesis...\n")

    final_summary = generate_final_summary(
        weekly_reports
    )

    final_document = f"""
        # BEP Engineering Dossier

        Generated:
        {datetime.now(timezone.utc).isoformat()}

        Model:
        {MODEL}

        Assessment Alignment:
        - Technical depth
        - Engineering methodology
        - Scientific rigor
        - Architectural reasoning
        - Reflection and justification
        - Professional engineering communication

        ---

        {final_summary}

        ---

        # Weekly Engineering Reports

        {"\n\n---\n\n".join(weekly_reports)}
        """

    with open(
        OUTPUT_FILE,
        "w",
        encoding="utf-8"
    ) as f:
        f.write(final_document)

    print(
        f"\nBEP dossier written to: {OUTPUT_FILE}"
    )


if __name__ == "__main__":
    main()