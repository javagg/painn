use iced::widget::{button, column, container, pick_list, row, scrollable, slider, text};
use iced::{Element, Length};
use iced_aw::menu;

// Menu bar builder
pub fn build_menu_bar<'a>(_app: &crate::App) -> Element<'a, crate::Message> {
    // File submenu
    let file_sub = menu::Menu::new(vec![
        menu::Item::new(
            button(text("New").size(14)).width(Length::Shrink).on_press(crate::Message::Clear),
        ),
        menu::Item::new(
            button(text("Open...").size(14)).width(Length::Shrink).on_press(crate::Message::OpenFile),
        ),
        menu::Item::new(
            button(text("Save").size(14)).width(Length::Shrink).on_press(crate::Message::SaveFile),
        ),
        menu::Item::new(
            button(text("Save As...").size(14)).width(Length::Shrink).on_press(crate::Message::SaveFileAs),
        ),
        menu::Item::new(
            button(text("Quit").size(14)).width(Length::Shrink).on_press(crate::Message::Quit),
        ),
    ]);
    let file_menu = menu::Item::with_menu(text("File").size(14), file_sub);

    // Edit submenu
    let edit_sub = menu::Menu::new(vec![
        menu::Item::new(button(text("Undo").size(14)).width(Length::Shrink).on_press(crate::Message::Undo)),
        menu::Item::new(button(text("Redo").size(14)).width(Length::Shrink).on_press(crate::Message::Redo)),
        menu::Item::new(button(text("Cut").size(14)).width(Length::Shrink).on_press(crate::Message::Cut)),
        menu::Item::new(button(text("Copy").size(14)).width(Length::Shrink).on_press(crate::Message::Copy)),
        menu::Item::new(button(text("Paste").size(14)).width(Length::Shrink).on_press(crate::Message::Paste)),
        menu::Item::new(button(text("Delete Selected").size(14)).width(Length::Shrink).on_press(crate::Message::DeleteSelected)),
        menu::Item::new(button(text("Clear").size(14)).width(Length::Shrink).on_press(crate::Message::Clear)),
    ]);
    let edit_menu = menu::Item::with_menu(text("Edit").size(14), edit_sub);

    // View submenu
    let view_sub = menu::Menu::new(vec![
        menu::Item::new(button(text("Toggle 2D Grid").size(14)).width(Length::Shrink).on_press(crate::Message::ToggleGrid)),
        menu::Item::new(button(text("Reset Zoom").size(14)).width(Length::Shrink).on_press(crate::Message::ResetZoom)),
        menu::Item::new(button(text("Toggle Scene Grid").size(14)).width(Length::Shrink).on_press(crate::Message::SceneGridToggle)),
        menu::Item::new(button(text("Toggle Axes").size(14)).width(Length::Shrink).on_press(crate::Message::SceneAxesToggle)),
        menu::Item::new(button(text("Clear Camera Align").size(14)).width(Length::Shrink).on_press(crate::Message::SceneCameraPresetCleared)),
    ]);
    let view_menu = menu::Item::with_menu(text("View").size(14), view_sub);

    // Help submenu
    let help_sub = menu::Menu::new(vec![
        menu::Item::new(button(text("About").size(14)).width(Length::Shrink).on_press(crate::Message::About)),
    ]);
    let help_menu = menu::Item::with_menu(text("Help").size(14), help_sub);

    menu::MenuBar::new(vec![file_menu, edit_menu, view_menu, help_menu]).into()
}

// Tab button builder
pub fn tab_button<'a>(label: &'a str, tab: crate::Tab, active: crate::Tab) -> iced::widget::Button<'a, crate::Message> {
    if tab == active {
        button(text(format!("● {label}"))).on_press_maybe(None)
    } else {
        button(text(label)).on_press(crate::Message::TabChanged(tab))
    }
}

// Scene sidebar: entity list with actions
pub fn build_scene_sidebar<'a>(entities: &[crate::scene::SceneEntityInfo]) -> Element<'a, crate::Message> {
    let list = entities.iter().map(|e| {
        let _pos = e.position;
        row![
            text(format!("#{}", e.id)).width(Length::Fixed(60.0)),
            text(match e.kind { crate::scene::SolidKind::Box => "Box", crate::scene::SolidKind::Sphere => "Sphere", crate::scene::SolidKind::Cylinder => "Cylinder", crate::scene::SolidKind::Cone => "Cone", crate::scene::SolidKind::Torus => "Torus" }).width(Length::Fixed(80.0)),
            // text(format!("pos: {:.2},{:.2},{:.2}", pos[0], pos[1], pos[2])).width(Length::Fixed(160.0)),
            text(format!("size: {:.2}", e.size)).width(Length::Fixed(80.0)),
            row![
                button(text("选中")).on_press(crate::Message::SceneSelectEntity(e.id)),
                button(text("聚焦")).on_press(crate::Message::SceneFocusEntity(e.id)),
            ].spacing(6),
        ]
        .spacing(8)
        .into()
    });

    let content = column(list).spacing(6).padding(8);
    container(scrollable(content))
        .width(Length::Fixed(360.0))
        .height(Length::Fill)
        .into()
}

// Draw tab toolbar
pub fn draw_toolbar<'a>(
    tool: crate::Tool,
    mode: crate::BooleanMode,
    stroke_width: f32,
    show_grid: bool,
    show_menubar: bool,
    selected: Option<usize>,
) -> Element<'a, crate::Message> {
    row![
        text("Tool"),
        pick_list(crate::Tool::ALL.as_slice(), Some(tool), crate::Message::ToolChanged)
            .width(Length::Fixed(120.0)),
        text("Mode"),
        pick_list(crate::BooleanMode::ALL.as_slice(), Some(mode), crate::Message::ModeChanged)
            .width(Length::Fixed(120.0)),
        text("Stroke"),
        slider(1.0..=16.0, stroke_width, crate::Message::StrokeWidthChanged)
            .width(Length::Fixed(180.0)),
        text(format!("{:.1}", stroke_width)),
        button(if show_grid { "Grid: On" } else { "Grid: Off" })
            .on_press(crate::Message::ToggleGrid),
        button("Reset Zoom").on_press(crate::Message::ResetZoom),
        button(if show_menubar { "MenuBar: On" } else { "MenuBar: Off" })
            .on_press(crate::Message::ToggleMenuBar),
        button("Undo").on_press(crate::Message::Undo),
        button("Delete Selected")
            .on_press_maybe(selected.map(|_| crate::Message::DeleteSelected)),
        button("Clear").on_press(crate::Message::Clear),
    ]
    .spacing(10)
    .align_y(iced::Alignment::Center)
    .into()
}

pub fn stroke_row<'a>() -> Element<'a, crate::Message> {
    row![
        text("Stroke"),
        button("Black").on_press(crate::Message::StrokeColorChanged(iced::Color::BLACK)),
        button("Red")
            .on_press(crate::Message::StrokeColorChanged(iced::Color::from_rgb8(0xE6, 0x2E, 0x2E))),
        button("Green")
            .on_press(crate::Message::StrokeColorChanged(iced::Color::from_rgb8(0x2E, 0xE6, 0x6B))),
        button("Blue")
            .on_press(crate::Message::StrokeColorChanged(iced::Color::from_rgb8(0x2E, 0xB8, 0xE6))),
    ]
    .spacing(8)
    .align_y(iced::Alignment::Center)
    .into()
}

pub fn fill_row<'a>() -> Element<'a, crate::Message> {
    row![
        text("Fill"),
        button("None").on_press(crate::Message::FillColorChanged(None)),
        button("Light Blue")
            .on_press(crate::Message::FillColorChanged(Some(iced::Color::from_rgb8(0xB3, 0xD9, 0xFF)))),
        button("Light Red")
            .on_press(crate::Message::FillColorChanged(Some(iced::Color::from_rgb8(0xFF, 0xB3, 0xB3)))),
    ]
    .spacing(8)
    .align_y(iced::Alignment::Center)
    .into()
}

pub fn status_row<'a>(
    tool: crate::Tool,
    zoom: f32,
    cursor_pos: Option<iced::Point>,
) -> Element<'a, crate::Message> {
    let hint = match tool {
        crate::Tool::Select => "Select: left click a shape; delete selected",
        crate::Tool::Spline => "Spline: left click to add points, right click to finish",
        crate::Tool::Polygon => "Polygon: left click to add points, right click to close",
        _ => "Drag to draw: press-move-release",
    };

    row![
        text(format!("Zoom: {:.0}%", zoom * 100.0)),
        text("|"),
        text(match cursor_pos {
            Some(p) => format!("Coords: {:.1}, {:.1}", p.x, p.y),
            None => String::from("Coords: -, -"),
        }),
        text("|"),
        text(hint),
    ]
    .spacing(10)
    .align_y(iced::Alignment::Center)
    .into()
}

pub fn scene_controls<'a>(
    scene_show_grid: bool,
    scene_grid_plane: crate::GridPlane,
    scene_tool: crate::SceneTool,
    scene_camera_mode: crate::CameraMode,
    scene_camera_preset: Option<crate::CameraPreset>,
    scene_axes_enabled: bool,
    scene_axes_size: f32,
    scene_axes_margin: f32,
    scene_grid_extent: f32,
    scene_grid_step: f32,
) -> Element<'a, crate::Message> {
    column![
        row![
            button(if scene_show_grid { "Grid: On" } else { "Grid: Off" })
                .on_press(crate::Message::SceneGridToggle),
            text("Plane"),
            pick_list(
                crate::GridPlane::ALL.as_slice(),
                Some(scene_grid_plane),
                crate::Message::SceneGridPlaneChanged,
            )
            .width(Length::Fixed(120.0)),
        ]
        .spacing(10)
        .align_y(iced::Alignment::Center),
        row![
            text("Primitive"),
            pick_list(
                crate::SceneTool::ALL.as_slice(),
                Some(scene_tool),
                crate::Message::SceneToolChanged,
            )
            .width(Length::Fixed(160.0)),
            text("Create with left click + drag"),
        ]
        .spacing(10)
        .align_y(iced::Alignment::Center),
        row![
            text("Camera"),
            pick_list(
                crate::CameraMode::ALL.as_slice(),
                Some(scene_camera_mode),
                crate::Message::SceneCameraModeChanged,
            )
            .width(Length::Fixed(160.0)),
            text("Align"),
            pick_list(
                crate::CameraPreset::ALL.as_slice(),
                scene_camera_preset,
                crate::Message::SceneCameraPresetChanged,
            )
            .width(Length::Fixed(160.0)),
            button("Clear Align").on_press(crate::Message::SceneCameraPresetCleared),
        ]
        .spacing(10)
        .align_y(iced::Alignment::Center),
        row![
            button(if scene_axes_enabled { "Axes: On" } else { "Axes: Off" })
                .on_press(crate::Message::SceneAxesToggle),
            text("Size"),
            slider(24.0..=240.0, scene_axes_size, crate::Message::SceneAxesSizeChanged)
                .width(Length::Fixed(200.0)),
            text(format!("{:.0}", scene_axes_size)),
            text("Margin"),
            slider(0.0..=64.0, scene_axes_margin, crate::Message::SceneAxesMarginChanged)
                .width(Length::Fixed(160.0)),
            text(format!("{:.0}", scene_axes_margin)),
        ]
        .spacing(10)
        .align_y(iced::Alignment::Center),
        row![
            text("Range"),
            slider(0.5..=10.0, scene_grid_extent, crate::Message::SceneGridExtentChanged)
                .width(Length::Fixed(200.0)),
            text(format!("{:.2}", scene_grid_extent)),
            text("Density"),
            slider(0.05..=2.0, scene_grid_step, crate::Message::SceneGridStepChanged)
                .width(Length::Fixed(200.0)),
            text(format!("{:.2}", scene_grid_step)),
        ]
        .spacing(10)
        .align_y(iced::Alignment::Center),
    ]
    .spacing(8)
    .into()
}
