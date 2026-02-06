use iced::Size;

use pain::App;

fn main() -> iced::Result {
    iced::application(App::new, App::update, App::view)
        .title("2D Vector Drawing Tool")
        .window_size(Size::new(1200.0, 800.0))
        .run()
}
