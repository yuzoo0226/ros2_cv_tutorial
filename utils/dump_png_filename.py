import os
import cv2
import numpy as np


def list_images_and_save_to_txt(folder_path, output_txt_path):
    # 画像ファイルの拡張子を指定
    image_extensions = ('.png', '.jpg', '.jpeg', '.bmp', '.tiff', '.gif')

    # フォルダ内の画像ファイルを取得し、ソート
    image_files = sorted([f for f in os.listdir(folder_path) if f.lower().endswith(image_extensions)])

    # 画像ファイル名をtxtファイルに書き込み
    with open(output_txt_path, 'w') as txt_file:
        for image_file in image_files:
            image_file = os.path.splitext(os.path.basename(image_file))[0]
            txt_file.write(image_file + '\n')

    print(f"画像ファイル名を '{output_txt_path}' に書き込みました。")


def rename_and_sort_png_files(folder_path):
    # 画像ファイルの拡張子を指定（ここではpngのみ）
    image_extension = '.png'

    # フォルダ内のpngファイルを取得し、ソート
    image_files = sorted([f for f in os.listdir(folder_path) if f.lower().endswith(image_extension)])

    # 画像ファイルを順番にリネーム
    for idx, image_file in enumerate(image_files):
        # 新しいファイル名を作成
        new_file_name = f"frame_{idx+1:05d}.png"

        # 古いファイルパスと新しいファイルパス
        old_file_path = os.path.join(folder_path, image_file)
        new_file_path = os.path.join(folder_path, new_file_name)

        # ファイルをリネーム
        os.rename(old_file_path, new_file_path)
        print(f"'{image_file}' を '{new_file_name}' にリネームしました。")


def file_name_change(folder_path, depth_path):
    # 画像ファイルの拡張子を指定
    image_extensions = ('.png', '.jpg', '.jpeg', '.bmp', '.tiff', '.gif')

    # フォルダ内の画像ファイルを取得し、ソート
    image_files = sorted([f for f in os.listdir(folder_path) if f.lower().endswith(image_extensions)])

    # 画像ファイル名をtxtファイルに書き込み
    with open(output_txt_path, 'w') as txt_file:
        for image_file in image_files:
            txt_file.write(image_file + '\n')


def create_black_images_from_source(images_folder, labels_folder):
    # imagesフォルダの全画像ファイルを取得
    image_files = [f for f in os.listdir(images_folder) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff', '.gif'))]

    # labelsフォルダが存在しない場合は作成
    if not os.path.exists(labels_folder):
        os.makedirs(labels_folder)

    # 各画像を読み込み、同じサイズの黒い画像を作成してlabelsフォルダに保存
    for image_file in image_files:
        # 画像の読み込み
        image_path = os.path.join(images_folder, image_file)
        image = cv2.imread(image_path)

        # 画像と同じサイズの黒い画像を作成（全ピクセルが0の画像）
        black_image = np.zeros_like(image)
        black_image = cv2.cvtColor(black_image, cv2.COLOR_BGR2GRAY)

        # 保存先のパスを作成
        label_path = os.path.join(labels_folder, image_file)

        # 黒い画像を保存
        cv2.imwrite(label_path, black_image)

        print(f"'{image_file}' の黒い画像を '{label_path}' に保存しました。")


def crop_images_center(images_folder, output_folder, crop_width=640, crop_height=480, is_depth=False, is_label=False):
    # 出力フォルダが存在しない場合は作成
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # 画像ファイルの拡張子を指定
    image_extensions = ('.png', '.jpg', '.jpeg', '.bmp', '.tiff')

    # imagesフォルダ内の画像を取得
    image_files = [f for f in os.listdir(images_folder) if f.lower().endswith(image_extensions)]

    for image_file in image_files:
        # 画像を読み込む
        image_path = os.path.join(images_folder, image_file)
        if is_depth:
            image = cv2.imread(image_path, cv2.IMREAD_ANYDEPTH)
        if is_label:
            image = cv2.imread(image_path, 0)
        else:
            image = cv2.imread(image_path)

        # 画像のサイズを取得
        height, width = image.shape[:2]

        # クロップ領域の開始位置を計算（画像中心から640x480を切り抜く）
        start_x = max(0, (width - crop_width) // 2)
        start_y = max(0, (height - crop_height) // 2)

        # クロップ処理
        cropped_image = image[start_y:start_y + crop_height, start_x:start_x + crop_width]

        # 出力先のファイルパス
        output_path = os.path.join(output_folder, image_file)

        # クロップした画像を保存
        cv2.imwrite(output_path, cropped_image)
        print(f"'{image_file}' をクロップして '{output_path}' に保存しました。")
