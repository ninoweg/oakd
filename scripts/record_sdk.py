from depthai_sdk import OakCamera, RecordType

with OakCamera() as oak:
    color = oak.create_camera('color', resolution='1080P', fps=30, encode='H265')
    left = oak.create_camera('left', resolution='720p', fps=30, encode='H265')
    right = oak.create_camera('right', resolution='720p', fps=30, encode='H265')

    # Synchronize & save all (encoded) streams
    oak.record([color.out.encoded, left.out.encoded, right.out.encoded], './', RecordType.VIDEO)
    # Show color stream
    # oak.visualize([color.out.camera], scale=2/3, fps=True)

    oak.start(blocking=True)