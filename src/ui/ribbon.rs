use iced::widget::{button, column, container, row, text};
use iced::{Alignment, Element, Length, Padding};
use iced_aw::{TabBar, TabLabel};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RibbonTab {
	Home,
	Sketch,
	Solid,
	View,
}

impl RibbonTab {
	pub const ALL: [RibbonTab; 4] = [
		RibbonTab::Home,
		RibbonTab::Sketch,
		RibbonTab::Solid,
		RibbonTab::View,
	];

	pub fn label(self) -> &'static str {
		match self {
			RibbonTab::Home => "Home",
			RibbonTab::Sketch => "Sketch",
			RibbonTab::Solid => "Solid",
			RibbonTab::View => "View",
		}
	}
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RibbonAction {
	New,
	Open,
	Save,
	Undo,
	Redo,
	Line,
	Rect,
	Circle,
	Extrude,
	Revolve,
	Fillet,
	Chamfer,
	Measure,
	Section,
	TopView,
	IsoView,
}

#[derive(Debug, Clone)]
pub struct RibbonButton {
	pub icon: &'static str,
	pub label: &'static str,
	pub action: RibbonAction,
}

#[derive(Debug, Clone)]
pub struct RibbonGroup {
	pub title: &'static str,
	pub buttons: Vec<RibbonButton>,
}

pub fn default_groups(tab: RibbonTab) -> Vec<RibbonGroup> {
	match tab {
		RibbonTab::Home => vec![
			RibbonGroup {
				title: "File",
				buttons: vec![
					RibbonButton {
						icon: "📄",
						label: "New",
						action: RibbonAction::New,
					},
					RibbonButton {
						icon: "📂",
						label: "Open",
						action: RibbonAction::Open,
					},
					RibbonButton {
						icon: "💾",
						label: "Save",
						action: RibbonAction::Save,
					},
				],
			},
			RibbonGroup {
				title: "Edit",
				buttons: vec![
					RibbonButton {
						icon: "↶",
						label: "Undo",
						action: RibbonAction::Undo,
					},
					RibbonButton {
						icon: "↷",
						label: "Redo",
						action: RibbonAction::Redo,
					},
				],
			},
		],
		RibbonTab::Sketch => vec![RibbonGroup {
			title: "Sketch",
			buttons: vec![
				RibbonButton {
					icon: "／",
					label: "Line",
					action: RibbonAction::Line,
				},
				RibbonButton {
					icon: "▭",
					label: "Rect",
					action: RibbonAction::Rect,
				},
				RibbonButton {
					icon: "◯",
					label: "Circle",
					action: RibbonAction::Circle,
				},
			],
		}],
		RibbonTab::Solid => vec![
			RibbonGroup {
				title: "Create",
				buttons: vec![
					RibbonButton {
						icon: "⬆",
						label: "Extrude",
						action: RibbonAction::Extrude,
					},
					RibbonButton {
						icon: "⟳",
						label: "Revolve",
						action: RibbonAction::Revolve,
					},
				],
			},
			RibbonGroup {
				title: "Modify",
				buttons: vec![
					RibbonButton {
						icon: "◝",
						label: "Fillet",
						action: RibbonAction::Fillet,
					},
					RibbonButton {
						icon: "⟂",
						label: "Chamfer",
						action: RibbonAction::Chamfer,
					},
				],
			},
		],
		RibbonTab::View => vec![
			RibbonGroup {
				title: "Inspect",
				buttons: vec![
					RibbonButton {
						icon: "📏",
						label: "Measure",
						action: RibbonAction::Measure,
					},
					RibbonButton {
						icon: "⟂",
						label: "Section",
						action: RibbonAction::Section,
					},
				],
			},
			RibbonGroup {
				title: "Camera",
				buttons: vec![
					RibbonButton {
						icon: "⬛",
						label: "Top",
						action: RibbonAction::TopView,
					},
					RibbonButton {
						icon: "◇",
						label: "Iso",
						action: RibbonAction::IsoView,
					},
				],
			},
		],
	}
}

pub fn ribbon<'a, Message: Clone + 'static>(
	active_tab: RibbonTab,
	on_tab_select: fn(RibbonTab) -> Message,
	on_action: fn(RibbonAction) -> Message,
) -> Element<'a, Message> {
	let mut tab_bar = TabBar::new(on_tab_select);
	for tab in RibbonTab::ALL {
		tab_bar = tab_bar.push(tab, TabLabel::Text(tab.label().to_string()));
	}
	tab_bar = tab_bar.set_active_tab(&active_tab);

	let groups = default_groups(active_tab)
		.into_iter()
		.map(|group| ribbon_group(group, &on_action))
		.collect::<Vec<_>>();

	let groups_row = row(groups)
		.spacing(12)
		.align_y(Alignment::Center);

	column![tab_bar, groups_row]
		.spacing(8)
		.padding(Padding::new(8.0))
		.width(Length::Fill)
		.into()
}

fn ribbon_group<'a, Message: Clone + 'static>(
	group: RibbonGroup,
	on_action: &impl Fn(RibbonAction) -> Message,
) -> Element<'a, Message> {
	let buttons = group
		.buttons
		.into_iter()
		.map(|b| ribbon_button(b, on_action))
		.collect::<Vec<_>>();

	let content = column![
		row(buttons)
			.spacing(8)
			.align_y(Alignment::Center),
		text(group.title).size(12),
	]
	.spacing(6)
	.align_x(Alignment::Center);

	container(content)
		.padding(Padding::new(8.0))
		.width(Length::Shrink)
		.into()
}

fn ribbon_button<'a, Message: Clone + 'static>(
	button_def: RibbonButton,
	on_action: &impl Fn(RibbonAction) -> Message,
) -> Element<'a, Message> {
	let content = column![
		text(button_def.icon).size(20),
		text(button_def.label).size(12),
	]
	.spacing(4)
	.align_x(Alignment::Center);

	button(content)
		.on_press(on_action(button_def.action))
		.width(Length::Fixed(72.0))
		.height(Length::Fixed(64.0))
		.into()
}
