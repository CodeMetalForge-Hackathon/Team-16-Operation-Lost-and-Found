import pandas as pd

df = pd.read_csv("data/metadata.csv")

print("Total images:", len(df))

print("\nSubject labels:")
print(df["subject_type"].value_counts())

print("\nOcclusion levels:")
print(df["occlusion_level"].value_counts())

print("\nLabel combinations:")
print(pd.crosstab(
    df["subject_type"],
    df["occlusion_level"]
))

print("\nRecording sessions:")
print(df["session_id"].nunique())