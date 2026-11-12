from types import SimpleNamespace

from builtin_interfaces.msg import Time

from gimbal_tf_broadcaster import GimbalTfBroadcaster


def test_optional_muzzle_transform_uses_measured_pitch_frame_offset():
    published = []
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time())
        ),
        _make_transform=GimbalTfBroadcaster._make_transform,
        pitch_frame='pitch_joint',
        camera_link_frame='camera_link',
        camera_optical_frame='camera_optical_frame',
        pitch_to_camera=(0.1, 0.0, 0.02),
        camera_mount_rpy=(0.0, 0.0, 0.0),
        muzzle_frame='muzzle',
        pitch_to_muzzle=(0.2, -0.01, 0.03),
        static_broadcaster=SimpleNamespace(sendTransform=published.append),
    )
    GimbalTfBroadcaster._publish_static_transforms(node)

    transforms = {item.child_frame_id: item for item in published[0]}
    assert set(transforms) == {'camera_link', 'camera_optical_frame', 'muzzle'}
    muzzle = transforms['muzzle']
    assert muzzle.header.frame_id == 'pitch_joint'
    assert (muzzle.transform.translation.x, muzzle.transform.translation.y,
            muzzle.transform.translation.z) == (0.2, -0.01, 0.03)
    assert muzzle.transform.rotation.w == 1.0

    node.muzzle_frame = ''
    published.clear()
    GimbalTfBroadcaster._publish_static_transforms(node)
    assert {item.child_frame_id for item in published[0]} == {
        'camera_link', 'camera_optical_frame'
    }
