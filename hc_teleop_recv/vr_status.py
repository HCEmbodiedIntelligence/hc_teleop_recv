"""A current display snapshot, derived from the same state that gates output."""


def control_status(frontend, now, input_ready=True):
    config, packet = frontend.config, frontend.packet
    fresh = packet is not None and now - frontend.input_at <= config.input_timeout
    channels = {}
    for ident, channel in frontend.channels.items():
        cfg = channel.config
        fk_fresh = channel.fk is not None and now - channel.fk_at <= config.fk_timeout
        tracked = fresh and packet.protocol_version == 2 and all(
            packet.tracked(hand) for hand in (cfg.controller, cfg.clutch_controller))
        grip = fresh and getattr(packet, cfg.clutch_controller + '_input').grip >= cfg.clutch_threshold
        if frontend.inhibited:
            state, reason = 'motion_active', '回位执行中，手柄控制已暂停'
        elif not frontend.enabled:
            state, reason = 'disabled', '遥操作未启用，请按 A'
        elif not config.arm_control_enabled:
            state, reason = 'disabled', '机械臂遥操作已关闭'
        elif not fresh:
            state, reason = 'input_timeout', '手柄输入已断流，等待重新连接'
        elif not input_ready:
            state, reason = 'waiting', '正在清理积压输入，暂停输出'
        elif not tracked:
            state, reason = 'tracking_lost', '手柄跟踪不可用'
        elif not fk_fresh:
            state, reason = 'feedback_timeout', '等待机械臂的新鲜位置反馈'
        elif not grip:
            hand = '左手' if cfg.clutch_controller == 'left' else '右手'
            state, reason = 'ready', f'遥操作已启用，请握住{hand}握持键'
        elif channel.state == 'active':
            state, reason = 'active', '正在发送遥操作目标'
        else:
            state, reason = 'waiting', channel.reason
        channels[ident] = {'state': state, 'reason': reason, 'fk_fresh': fk_fresh,
                           'tracked': tracked, 'grip': grip}
    if frontend.inhibited:
        state, reason = 'motion_active', '回位执行中，手柄控制已暂停'
    elif not frontend.enabled:
        state, reason = 'disabled', '遥操作未启用，请按 A'
    elif not fresh:
        state, reason = 'input_timeout', '手柄输入已断流，等待重新连接'
    elif not input_ready:
        state, reason = 'waiting', '正在清理积压输入，暂停输出'
    elif config.arm_control_enabled and channels:
        usable = [c for c in channels.values() if c['state'] in ('active', 'ready')]
        if usable and len(usable) < len(channels):
            state, reason = 'partial', '部分机械臂暂不可控，请检查各臂状态'
        else:
            selected = next((c for c in usable if c['state'] == 'active'),
                            (usable or list(channels.values()))[0])
            state, reason = selected['state'], selected['reason']
    else:
        state, reason = 'enabled', '遥操作已启用；机械臂输出已关闭'
    return {'state': state, 'reason': reason, 'enabled': frontend.enabled,
            'motion_active': frontend.inhibited, 'arm_control_enabled': config.arm_control_enabled,
            'input_fresh': fresh, 'channels': channels}
