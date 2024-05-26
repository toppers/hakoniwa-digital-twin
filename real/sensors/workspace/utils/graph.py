import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_segments(file_path):
    # CSVファイルを読み込む
    df = pd.read_csv(file_path, header=None, names=['Segment', 'Index', 'Degree', 'Value'])

    # Degreeをラジアンに変換
    df['Degree_rad'] = df['Degree']

    # x, y の計算
    df['x'] = df['Value'] * np.cos(df['Degree_rad'])
    df['y'] = df['Value'] * np.sin(df['Degree_rad'])

    # データをプロットする
    plt.figure(figsize=(10, 6))
    scatter = plt.scatter(df['x'], df['y'], c=df['Segment'], cmap='viridis', marker='o')
    plt.colorbar(scatter, label='Segment')
    plt.xlabel('x (Value * cos(Degree))')
    plt.ylabel('y (Value * sin(Degree))')
    plt.title('Segments Scatter Plot (x = Value * cos(Degree), y = Value * sin(Degree))')
    plt.grid(True)
    plt.show()

# ファイルパスを指定してプロット関数を呼び出す
file_path = 'segments.csv'
plot_segments(file_path)
