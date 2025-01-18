import disnake
from disnake.ext import commands

intents = disnake.Intents.default()
intents.members = True

bot = commands.Bot(command_prefix="!>", intents=intents)

@bot.event
async def on_ready():
    print(f"Мы вошли как {bot.user}")
    await bot.change_presence(status=disnake.Status.dnd, activity=disnake.Game('LifeMC | 1 Сезон'))

activity = disnake.Activity(
    name="LifeMC | 1 Сезон",
    type=disnake.ActivityType.playing,
)

@bot.event
async def on_ready():
    print(" ---- ----- [---] |---| -----")
    print(" |      |   |   | |   |   |")                  
    print(" ----   |   |---| |---|   |")
    print("    |   |   |   | |\      |")
    print(" ----   |   |   | | \     |")
    print(f'Logged in as {bot.user}')

bot.load_extensions("cogs")


bot.run("")
