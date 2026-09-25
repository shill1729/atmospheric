import pandas as pd
import re
from pathlib import Path


# Return the source metadata lines of an export as one string.
def read_source_metadata(filepath):
    lines = []

    with open(filepath, "r") as file:
        for line in file:
            if not line.startswith("#"):
                break

            if line.startswith("# Sources:") or line.startswith("# Source "):
                lines.append(line.rstrip())

    return "\n".join(lines)



# Parse the "# Source N:" lines into a DataFrame. Positions are lat/lon when the
# export is georeferenced (the NY dataset was present) and x/y domain metres otherwise.
SOURCE_PATTERN = re.compile(
    r"""
    ^\#\s*Source\s+(?P<source_id>\d+):\s*
    (?:lat=(?P<lat>[+-]?\d+(?:\.\d+)?)\s+lon=(?P<lon>[+-]?\d+(?:\.\d+)?)
      |x=(?P<x>[+-]?\d+(?:\.\d+)?)\s+y=(?P<y>[+-]?\d+(?:\.\d+)?))\s+
    born=(?P<born>[+-]?\d+(?:\.\d+)?)s\s+
    lifespan=(?P<lifespan>[+-]?\d+(?:\.\d+)?)s\s+
    died=(?P<died>[+-]?\d+(?:\.\d+)?)s
    """,
    re.VERBOSE,
)


def get_source_metadata(filepath: str | Path) -> pd.DataFrame:
    sources = []

    with open(filepath, "r", encoding="utf-8") as file:
        for line in file:
            # Metadata ends when the CSV header begins.
            if not line.startswith("#"):
                break

            match = SOURCE_PATTERN.match(line.strip())
            if match is not None:
                source = match.groupdict()
                row = {"source_id": int(source["source_id"])}
                if source["lat"] is not None:
                    row["lat_deg"] = float(source["lat"])
                    row["lon_deg"] = float(source["lon"])
                else:
                    row["x_m"] = float(source["x"])
                    row["y_m"] = float(source["y"])
                row["born_s"] = float(source["born"])
                row["died_s"] = float(source["died"])
                sources.append(row)

    return pd.DataFrame(sources)

if __name__ == "__main__":
    
    # Read the data rows, skipping the variable-length metadata header.
    filepath = "synthetic_pm25_data_1.csv"
    dat = pd.read_csv(filepath, comment="#")
    print(dat.head())
    # Print the source metadata as text.
    print(read_source_metadata(filepath))
    # Parse the source metadata into a DataFrame.
    md = get_source_metadata(filepath)
    print(md)
