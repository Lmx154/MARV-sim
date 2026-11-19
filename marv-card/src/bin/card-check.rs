use marv_card::{Card, Env, check_home_override};
use std::path::Path;
use std::process::ExitCode;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() != 2 {
        eprintln!("usage: card-check <card.toml> <env.toml>");
        return ExitCode::from(2);
    }
    let mut errors = Vec::new();
    let card = Card::load(&args[0]);
    let env = Env::load(&args[1]);
    if let Err(e) = &card {
        errors.extend(e.iter().cloned());
    }
    match &env {
        Err(e) => errors.extend(e.iter().cloned()),
        Ok(env) => {
            if let Err(e) = check_home_override(Path::new(&args[1]), &env.home_override) {
                errors.push(e);
            }
        }
    }
    if let (true, Ok(card), Ok(env)) = (errors.is_empty(), &card, &env) {
        println!("PASS");
        println!("open items in {}:", args[0]);
        for line in card.open_items() {
            println!("  {line}");
        }
        println!("open items in {}:", args[1]);
        for line in env.open_items() {
            println!("  {line}");
        }
        return ExitCode::SUCCESS;
    }
    for e in &errors {
        println!("{e}");
    }
    ExitCode::from(1)
}
