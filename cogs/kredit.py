import disnake
from disnake.ext import commands
from disnake import Option
import json
import os

class CreditRating(commands.Cog):
    def __init__(self, bot):
        self.bot = bot
        self.filename = 'credit_scores.json'
        self.credit_scores = self.load_credit_scores()

    def load_credit_scores(self):
        if os.path.exists(self.filename):
            try:
                with open(self.filename, 'r') as f:
                    return json.load(f)
            except json.JSONDecodeError:
                print("Ошибка при чтении данных из файла. Загружаем пустой словарь.")
                return {} 
        return {} 

    def save_credit_scores(self):
        try:
            with open(self.filename, 'w') as f:
                json.dump(self.credit_scores, f, indent=4)
        except IOError as e:
            print(f"Ошибка при сохранении файла: {e}")

    @commands.slash_command(description="Отображает кредитные рейтинги всех участников (без ботов).")
    async def credit_rating(self, inter):

        members = [member for member in inter.guild.members if not member.bot]

        embed = disnake.Embed(title="Кредитный рейтинг участников", color=disnake.Color.blue())

        for member in members:
            credit_score = self.credit_scores.get(str(member.id), 100) 
            embed.add_field(name=member.name, value=f"Кредитный рейтинг: {credit_score}", inline=False)

        embed.set_footer(text=f"Запросил: {inter.author.name}", icon_url=inter.author.avatar.url)
        await inter.send(embed=embed)

    @commands.slash_command(description="Добавляет кредитный рейтинг участнику.")
    async def add_credit(self, inter, member: disnake.Member = Option(name="member", description="Участник", required=True), amount: int = Option(name="amount", description="Количество, на которое увеличим рейтинг", required=True)):

        if member.bot:
            await inter.send("Ботам нельзя добавлять кредитный рейтинг.")
            return


        current_score = self.credit_scores.get(str(member.id), 100) 
        new_score = current_score + amount
        self.credit_scores[str(member.id)] = new_score


        self.save_credit_scores()


        embed = disnake.Embed(
            title=f"Кредитный рейтинг {member.name} обновлен",
            description=f"Рейтинг был увеличен на {amount}.",
            color=disnake.Color.green()
        )
        embed.add_field(name="Новый кредитный рейтинг:", value=f"**{new_score}**", inline=False)
        embed.set_footer(text=f"Запросил: {inter.author.name}", icon_url=inter.author.avatar.url)

        await inter.send(embed=embed)

    @commands.slash_command(description="Снимает кредитный рейтинг с участника.")
    async def remove_credit(self, inter, member: disnake.Member = Option(name="member", description="Участник", required=True), amount: int = Option(name="amount", description="Количество, на которое уменьшим рейтинг", required=True)):

        if member.bot:
            await inter.send("Ботам нельзя снимать кредитный рейтинг.")
            return


        current_score = self.credit_scores.get(str(member.id), 100)  
        new_score = max(0, current_score - amount)  
        self.credit_scores[str(member.id)] = new_score


        self.save_credit_scores()


        embed = disnake.Embed(
            title=f"Кредитный рейтинг {member.name} обновлен",
            description=f"Рейтинг был уменьшен на {amount}.",
            color=disnake.Color.red()
        )
        embed.add_field(name="Новый кредитный рейтинг:", value=f"**{new_score}**", inline=False)
        embed.set_footer(text=f"Запросил: {inter.author.name}", icon_url=inter.author.avatar.url)

        await inter.send(embed=embed)

    @commands.slash_command(description="Запрашивает кредитный рейтинг участника.")
    async def query_credit(self, inter, member: disnake.Member = Option(name="member", description="Участник", required=True)):



        credit_score = self.credit_scores.get(str(member.id), 100) 


        embed = disnake.Embed(
            title=f"Кредитный рейтинг {member.name}",
            description=f"Кредитный рейтинг участника: **{credit_score}**",
            color=disnake.Color.blue()
        )
        embed.set_footer(text=f"Запросил: {inter.author.name}", icon_url=inter.author.avatar.url)

        await inter.send(embed=embed)


def setup(bot):
    bot.add_cog(CreditRating(bot))
