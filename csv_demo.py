import pandas as pd
import re
from pathlib import Path

import pandas as pd

# Function used for reading in the source metadata and returning it as a string to print.
def read_source_metadata(filepath):
    lines = []

    with open(filepath, "r") as file:
        for line in file:
            if not line.startswith("#"):
                break

            if line.startswith("# Sources:") or line.startswith("# Source "):
                lines.append(line.rstrip())

    return "\n".join(lines)



# Alternatively actually parse the metadata and put it into a dataframe, using Regular Expressions
SOURCE_PATTERN = re.compile(
    r"""
    ^\#\s*Source\s+(?P<source_id>\d+):\s*
    lat=(?P<lat>[+-]?\d+(?:\.\d+)?)\s+
    lon=(?P<lon>[+-]?\d+(?:\.\d+)?)\s+
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
                sources.append(
                    {
                        "source_id": int(source["source_id"]),
                        "lat_deg": float(source["lat"]),
                        "lon_deg": float(source["lon"]),
                        "born_s": float(source["born"]),
                        "died_s": float(source["died"]),
                    }
                )

    return pd.DataFrame(sources)

if __name__ == "__main__":
    
    # Read in the file skipping the metadata (which has a variable amount of rows!)
    filepath = "synthetic_pm25_data_1.csv"
    dat = pd.read_csv(filepath, comment="#")
    print(dat.head())
    # Read and just print metadata as one big string
    print(read_source_metadata(filepath))
    # Actually get source metadata into a dataframe
    md = get_source_metadata(filepath)
    print(md)
