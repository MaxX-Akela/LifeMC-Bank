import disnake
from disnake.ext import commands
from disnake import Option
import json
import os

class BankBalance(commands.Cog):
    def __init__(self, bot):
        self.bot = bot
        self.filename = 'bank_balances.json'
        self.balances = self.load_balances()

    def load_balances(self):
        if os.path.exists(self.filename):
            try:
                with open(self.filename, 'r') as f:
                    return json.load(f)
            except json.JSONDecodeError:
                print("Ошибка при чтении данных из файла. Загружаем пустой словарь.")
                return {}  
        return {} 

    def save_balances(self):
        try:
            with open(self.filename, 'w') as f:
                json.dump(self.balances, f, indent=4)
        except IOError as e:
            print(f"Ошибка при сохранении файла: {e}")

    @commands.slash_command(description="Добавляет деньги на счёт участника.")
    async def add_money(self, inter, member: disnake.Member = Option(name="member", description="Участник", required=True), amount: int = Option(name="amount", description="Сумма, на которую увеличим баланс", required=True)):
        """Команда для добавления денег на счёт участника."""
        
        if member.bot:
            await inter.send("Ботам нельзя добавлять деньги.")
            return

        current_balance = self.balances.get(str(member.id), 0) 
        new_balance = current_balance + amount
        self.balances[str(member.id)] = new_balance
        
        self.save_balances()

        embed = disnake.Embed(
            title=f"Баланс {member.name} обновлен",
            description=f"Баланс был увеличен на {amount} ₽.",
            color=disnake.Color.green()
        )
        embed.add_field(name="Новый баланс:", value=f"**{new_balance} ₽**", inline=False)
        embed.set_footer(text=f"Запросил: {inter.author.name}", icon_url=inter.author.avatar.url)
        
        await inter.send(embed=embed)

    @commands.slash_command(description="Снимает деньги с счёта участника.")
    async def remove_money(self, inter, member: disnake.Member = Option(name="member", description="Участник", required=True), amount: int = Option(name="amount", description="Сумма, на которую уменьшим баланс", required=True)):
        
        if member.bot:
            await inter.send("Ботам нельзя снимать деньги.")
            return

        # Получаем текущий баланс участника
        current_balance = self.balances.get(str(member.id), 0)  
        new_balance = max(0, current_balance - amount) 
        self.balances[str(member.id)] = new_balance
        
        self.save_balances()

        embed = disnake.Embed(
            title=f"Баланс {member.name} обновлен",
            description=f"Баланс был уменьшен на {amount} ₽.",
            color=disnake.Color.red()
        )
        embed.add_field(name="Новый баланс:", value=f"**{new_balance} ₽**", inline=False)
        embed.set_footer(text=f"Запросил: {inter.author.name}", icon_url=inter.author.avatar.url)
        
        await inter.send(embed=embed)

    @commands.slash_command(description="Запрашивает баланс участника.")
    async def query_balance(self, inter, member: disnake.Member = Option(name="member", description="Участник", required=True)):
        
        balance = self.balances.get(str(member.id), 0) 
        
        embed = disnake.Embed(
            title=f"Баланс {member.name}",
            description=f"Баланс участника: **{balance} ₽**",
            color=disnake.Color.blue()
        )
        embed.set_footer(text=f"Запросил: {inter.author.name}", icon_url=inter.author.avatar.url)
        
        await inter.send(embed=embed)

    @commands.slash_command(description="Переводит деньги от одного игрока к другому.")
    async def transfer_money(self, inter, from_member: disnake.Member = Option(name="from_member", description="От кого перевод", required=True), to_member: disnake.Member = Option(name="to_member", description="Кому перевод", required=True), amount: int = Option(name="amount", description="Сумма перевода", required=True)):
        
        if from_member.bot or to_member.bot:
            await inter.send("Ботам нельзя переводить деньги.")
            return
        
        if from_member == to_member:
            await inter.send("Нельзя перевести деньги самому себе.")
            return
        
        from_balance = self.balances.get(str(from_member.id), 0)
        to_balance = self.balances.get(str(to_member.id), 0)

        if from_balance < amount:
            await inter.send(f"У {from_member.name} недостаточно средств для перевода.")
            return

        self.balances[str(from_member.id)] = from_balance - amount
        self.balances[str(to_member.id)] = to_balance + amount

        self.save_balances()

        embed = disnake.Embed(
            title="Перевод денег завершён",
            description=f"{from_member.name} перевёл {amount} ₽ на счёт {to_member.name}.",
            color=disnake.Color.blue()
        )
        embed.add_field(name="Новый баланс отправителя:", value=f"**{from_balance - amount} ₽**", inline=False)
        embed.add_field(name="Новый баланс получателя:", value=f"**{to_balance + amount} ₽**", inline=False)
        embed.set_footer(text=f"Запросил: {inter.author.name}", icon_url=inter.author.avatar.url)

        await inter.send(embed=embed)

def setup(bot):
    bot.add_cog(BankBalance(bot))
