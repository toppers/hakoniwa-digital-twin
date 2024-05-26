import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_segments(file_path):
    # CSVファイルを読み込む
    df = pd.read_csv(file_path, header=None)
    
    # 列数を確認して列名を設定
    if df.shape[1] == 3:
        df.columns = ['Index', 'Degree', 'Value']
        df['Segment'] = 0  # オリジナルデータにはセグメント番号がないため、ダミー列を追加
    elif df.shape[1] == 4:
        df.columns = ['Segment', 'Index', 'Degree', 'Value']
    else:
        raise ValueError("CSVファイルの列数が不正です。")

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

if __name__ == '__main__':
    import argparse

    # 引数を解析する
    parser = argparse.ArgumentParser(description='Plot segments from a CSV file.')
    parser.add_argument('file_path', type=str, help='Path to the CSV file')
    args = parser.parse_args()

    # ファイルパスを指定してプロット関数を呼び出す
    plot_segments(args.file_path)
